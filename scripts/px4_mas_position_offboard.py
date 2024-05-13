#!/usr/bin/env python3

__author__ = "Minhyun Cho, Yifan Guo, Vishnu Vijay"
__contact__ = "@purdue.edu"

# python packages and modules
import argparse
import numpy as np
from functools import partial

# ros packages
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# px4/navigation packages
import navpy

# messages
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleGlobalPosition, VehicleCommand
from geometry_msgs.msg import PointStamped
from std_msgs.msg import UInt8, Bool, Float32MultiArray

class OffboardMission(Node):

    def __init__(self,n_drone,waypoints,formation_config,adjacency_matrix,ref_lla):

        # inheritance from parent class
        super().__init__("px4_offboard_mission")

        # set publisher and subscriber quality of service profile
        qos_profile_pub     =   QoSProfile(
            reliability     =   QoSReliabilityPolicy.BEST_EFFORT,
            durability      =   QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history         =   QoSHistoryPolicy.KEEP_LAST,
            depth           =   1
        )

        qos_profile_sub     =   QoSProfile(
            reliability     =   QoSReliabilityPolicy.BEST_EFFORT,
            durability      =   QoSDurabilityPolicy.VOLATILE,
            history         =   QoSHistoryPolicy.KEEP_LAST,
            depth           =   1
        )

        # define number of drones
        self.n_drone    =   n_drone

        # define subscribers and publishers
        self.array_publishers     =   [{'offboard_mode_pub':None, 'trajectory_pub':None, 'vehicle_command_pub':None, 'spawn_offset_pub':None} for _ in range(self.n_drone)]
        self.array_subscribers    =   [{'status_sub':None, 'local_pos_sub':None, 'global_pos_sub':None} for _ in range(self.n_drone)]
        self.element_publisher    =   [{'vleader_pos_pub':None, 'formation_pub':None, 'adjacency_pub':None}]

        for idx in range(self.n_drone):

            self.ns             =   f'/px4_{idx+1}'

            self.array_subscribers[idx]['status_sub']           =   self.create_subscription(
                VehicleStatus,
                f'{self.ns}/fmu/out/vehicle_status',
                partial(self.vehicle_status_callback,id=idx),                           # instead of lambda function, lambda msg: self.vehicle_status_callback(msg,id=i), use partial function
                qos_profile_sub)

            self.array_subscribers[idx]['local_pos_sub']        =   self.create_subscription(
                VehicleLocalPosition,
                f'{self.ns}/fmu/out/vehicle_local_position',
                partial(self.local_position_callback,id=idx),
                qos_profile_sub)

            self.array_subscribers[idx]['global_pos_sub']       =   self.create_subscription(
                VehicleGlobalPosition,
                f'{self.ns}/fmu/out/vehicle_global_position',
                partial(self.global_position_callback,id=idx),
                qos_profile_sub)

            self.array_publishers[idx]['offboard_mode_pub']     =   self.create_publisher(
                OffboardControlMode,
                f'{self.ns}/fmu/in/offboard_control_mode',
                qos_profile_pub)

            self.array_publishers[idx]['trajectory_pub']        =   self.create_publisher(
                TrajectorySetpoint,
                f'{self.ns}/fmu/in/trajectory_setpoint',
                qos_profile_pub)

            self.array_publishers[idx]['vehicle_command_pub']   =   self.create_publisher(
                VehicleCommand,
                f'{self.ns}/fmu/in/vehicle_command',
                qos_profile_pub)
            
            self.array_publishers[idx]['spawn_offset_pub']      =   self.create_publisher(
                PointStamped,
                f'{self.ns}/detector/spawn_offset',
                qos_profile_pub)
            
        self.element_publisher[0]['vleader_pos_pub']        =    self.create_publisher(
            PointStamped,
            '/px4_0/detector/vleader_position',
            qos_profile_pub)
        
        self.element_publisher[0]['formation_pub']          =    self.create_publisher(
            Float32MultiArray,
            '/px4_0/detector/formation_config',
            qos_profile_pub)
        
        self.element_publisher[0]['adjacency_pub']          =   self.create_publisher(
            Float32MultiArray,
            '/px4_0/detector/adjacency',
            qos_profile_pub)
                   
        # parameters for callback
        self.timer_period   =   0.02            # [sec] callback function frequency (offboardcontrolmode should be at least 2Hz)
        self.timer          =   self.create_timer(self.timer_period, self.cmdloop_callback)

        # get formation configuration and adjacency matrix of the vehicle network
        self.formation      =   formation_config
        self.adjacency      =   adjacency_matrix

        # set the reference as ned origin / convert global coordinates to local ones
        self.ref_lla        =   ref_lla
        self.wpt_idx        =   0

        self.wpts_lla       =   waypoints
        self.wpts_ned       =   navpy.lla2ned(self.wpts_lla[:,0], self.wpts_lla[:,1], self.wpts_lla[:,2],
                                              self.ref_lla[0], self.ref_lla[1], self.ref_lla[2],
                                              latlon_unit='deg', alt_unit='m', model='wgs84')

        # parameters for formation flight
        self.wpt_idx                =   np.uint8(0)
        
        self.velocity               =   np.float64(4.0, dtype=np.float64)           # [m/s] reference velocity of the vehicle
        self.norm                   =   np.float64(0.0, dtype=np.float64)           # [m] distance between each waypoint
        self.dir_vector             =   np.array([0.0,0.0,0.0], dtype=np.float64)   # [-] unit direction vector
        self.look_ahead             =   np.float64(0.0, dtype=np.float64)           # [m] look ahead distance
        self.nav_wpt_reach_thres    =   0.025*self.velocity                         # [m] waypoint reach condition radius

        # variables for virtual leader
        self.vleader_set_pt_ned     =   np.array([0.0,0.0,0.0], dtype=np.float64)
        self.vleader_prev_wpt_ned   =   np.array([0.0,0.0,0.0], dtype=np.float64)
        self.vleader_next_wpt_ned   =   np.array([0.0,0.0,0.0], dtype=np.float64)

        # variables for agents
        self.entry_execute      =   [False for _ in range(self.n_drone)]
        self.ned_spawn_offset   =   [None for _ in range(self.n_drone)]
        self.flight_phase       =   np.uint8(0)
        self.nav_state_list     =   [VehicleStatus.NAVIGATION_STATE_MAX for _ in range(self.n_drone)]
        self.next_phase_flag    =   False
        self.wpt_change_flag    =   False
        self.omega              =   np.float64(0.0)

        self.arm_counter_list       =   [0 for i in range(self.n_drone)]
        self.local_pos_ned_list     =   [None for _ in range(self.n_drone)]
        self.global_ref_lla_list    =   [None for _ in range(self.n_drone)]

        self.trajectory_set_pt      =   []
        self.yaw_set_pt             =   []
        for i in range(self.n_drone):
            self.trajectory_set_pt.append(np.array([0,0,0], dtype=np.float64))
            self.yaw_set_pt.append(np.array(0.0, dtype=np.float64))

        # attack vector
        self.attack_vector      =   []

        for i in range(self.n_drone):
            self.attack_vector.append(np.array([0,0,0], dtype=np.float64))

        # self.attack_vector[4]   =   self.formation[3,:]-self.formation[4,:]
        self.attack_vector[2]   =   0.7*self.formation[2,:]

        self.attack_duration    =   np.float64(10.0)
        self.attack_timer       =   np.float64(0.0)

    # subscriber callback
    def vehicle_status_callback(self,msg,id):
        self.nav_state_list[id] = msg.nav_state

    def local_position_callback(self,msg,id):
        self.local_pos_ned_list[id]     =   np.array([msg.x,msg.y,msg.z], dtype=np.float64)

    def global_position_callback(self,msg,id):
        self.global_ref_lla_list[id]    =   np.array([msg.lat,msg.lon,msg.alt], dtype=np.float64)

    def publish_vehicle_command(self,command,id,param1=0.0,param2=0.0,param3=0.0):
        msg                     =   VehicleCommand()
        msg.param1              =   param1
        msg.param2              =   param2
        msg.param3              =   param3
        msg.command             =   command     # command ID
        msg.target_system       =   0           # system which should execute the command
        msg.target_component    =   1           # component which should execute the command, 0 for all components
        msg.source_system       =   1           # system sending the command
        msg.source_component    =   1           # component sending the command
        msg.from_external       =   True
        msg.timestamp           =   int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.array_publishers[id]['vehicle_command_pub'].publish(msg)

    def publish_offboard_control_mode(self,id):
        msg                     =   OffboardControlMode()
        msg.timestamp           =   int(Clock().now().nanoseconds/1000)
        msg.position            =   True
        msg.velocity            =   False
        msg.acceleration        =   False
        msg.attitude            =   False
        msg.body_rate           =   False
        self.array_publishers[id]['offboard_mode_pub'].publish(msg)

    def publish_trajectory_setpoint(self,id):
        msg                     =   TrajectorySetpoint()
        msg.timestamp           =   int(Clock().now().nanoseconds/1000)
        msg.position            =   np.array(self.trajectory_set_pt[id], dtype=np.float32)
        msg.yaw                 =   float(self.yaw_set_pt[id])
        self.array_publishers[id]['trajectory_pub'].publish(msg)

    def publish_virleader_pos(self):

        msg                     =   PointStamped()
        msg.point.x             =   self.vleader_set_pt_ned[0]
        msg.point.y             =   self.vleader_set_pt_ned[1]
        msg.point.z             =   self.vleader_set_pt_ned[2]
        self.element_publisher[0]['vleader_pos_pub'].publish(msg)

    def publish_formation(self):
        msg                     =   Float32MultiArray()
        msg.data                =   (self.formation-self.ned_spawn_offset).flatten().tolist()
        self.element_publisher[0]['formation_pub'].publish(msg)

    def publish_adjacency(self):
        msg                     =   Float32MultiArray()
        msg.data                =   (self.adjacency).flatten().tolist()
        self.element_publisher[0]['adjacency_pub'].publish(msg)

    def publish_spawn_offset(self):

        for id in range(self.n_drone):
            msg                     =   PointStamped()
            msg.point.x             =   self.ned_spawn_offset[id][0]
            msg.point.y             =   self.ned_spawn_offset[id][1]
            msg.point.z             =   self.ned_spawn_offset[id][2]
            self.array_publishers[id]['spawn_offset_pub'].publish(msg)

    def cmdloop_callback(self):

        # flight phase 0: idle/arming/takeoff
        if self.flight_phase == 0:

            # entry:
            if all(not agent_entry for agent_entry in self.entry_execute):
                self.vleader_set_pt_ned     =   np.array([0.0,0.0,-5.0], dtype=np.float64)
                self.vleader_prev_wpt_ned   =   np.array([0.0,0.0,0.0], dtype=np.float64)
                self.vleader_next_wpt_ned   =   np.array([0.0,0.0,-5.0], dtype=np.float64)

            for idx in (idx for idx in range(self.n_drone) if self.entry_execute[idx] is False):
                # compute compensation of local position coordinate
                if self.global_ref_lla_list[idx] is not None and self.local_pos_ned_list[idx] is not None and self.entry_execute[idx] is False:
                    self.ned_spawn_offset[idx]  =   navpy.lla2ned(self.global_ref_lla_list[idx][0],self.global_ref_lla_list[idx][1],0,
                                                                self.ref_lla[0],self.ref_lla[1],self.ref_lla[2],
                                                                latlon_unit='deg', alt_unit='m', model='wgs84')

                    self.trajectory_set_pt[idx] =   np.copy(self.vleader_set_pt_ned)
                    self.yaw_set_pt[idx]        =   self.yaw_set_pt[idx]

                    self.publish_trajectory_setpoint(idx)

                    self.entry_execute[idx]     =   True

            # during:
            for idx in range(self.n_drone):
                if self.nav_state_list[idx] != VehicleStatus.ARMING_STATE_ARMED or self.arm_counter_list[idx] < 20:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,idx,1.0,6.0)
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,idx,1.0)
                    self.arm_counter_list[idx]  +=  1
                    # self.get_logger().info('Drone #'+str(idx+1)+' armed and dangerous....')

            for idx in (idx for idx in range(self.n_drone) if self.entry_execute[idx] is True):
                self.publish_offboard_control_mode(idx)
                self.publish_trajectory_setpoint(idx)

            # exit:
            if [True for idx in range(self.n_drone) if (self.local_pos_ned_list[idx] is not None)] == \
                [True for idx in range(self.n_drone)]:
                if [True for idx in range(self.n_drone) if (np.linalg.norm(self.trajectory_set_pt[idx]-self.local_pos_ned_list[idx]) < 1.0)] == \
                    [True for idx in range(self.n_drone)]:
                    self.next_phase_flag        =   True 


        elif self.flight_phase == 1:
                
            # entry:
            if all(not agent_entry for agent_entry in self.entry_execute):
                self.vleader_prev_wpt_ned   =   np.copy(self.vleader_next_wpt_ned)
                self.vleader_next_wpt_ned   =   np.copy(self.wpts_ned[self.wpt_idx])
                self.omega                  =   np.float64(0.0)

                self.norm                   =   np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)
                self.normvector             =   (self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)/self.norm

                for axis in range(3):
                    self.vleader_set_pt_ned[axis]   =   np.clip((1-self.omega)*self.vleader_prev_wpt_ned[axis] \
                                                                +self.omega*self.vleader_next_wpt_ned[axis], \
                                                                np.min(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])), \
                                                                np.max(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])))    

            # during:
            else:
                norm_delt       =   np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)/self.velocity
                self.omega      =   self.omega+self.timer_period/norm_delt
                self.omega      =   np.clip(self.omega,0,1)

                self.norm                   =   np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)
                self.normvector             =   (self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)/self.norm

                for axis in range(3):
                    self.vleader_set_pt_ned[axis]   =   np.clip((1-self.omega)*self.vleader_prev_wpt_ned[axis] \
                                                                +self.omega*self.vleader_next_wpt_ned[axis], \
                                                                np.min(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])), \
                                                                np.max(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])))    

            # entry/during:
            for idx in range(self.n_drone):

                self.trajectory_set_pt[idx]     =   np.copy(self.vleader_set_pt_ned)
                self.trajectory_set_pt[idx]     =   self.trajectory_set_pt[idx]+self.formation[idx,:]-self.ned_spawn_offset[idx] \
                                                    +self.look_ahead*self.normvector+self.attack_vector[idx]*self.attack_timer
                self.yaw_set_pt[idx]            =   self.yaw_set_pt[idx]

                self.publish_offboard_control_mode(idx)
                self.publish_trajectory_setpoint(idx)

                if self.entry_execute[idx] is False:
                    self.entry_execute[idx]     =   True

            print('error of the agent 2')
            print(self.trajectory_set_pt[2]-self.vleader_set_pt_ned-self.formation[2,:]+self.ned_spawn_offset[2])
            print('true error of agent2')
            print(self.attack_vector[2]*self.attack_timer)

            if self.wpt_idx >= 2:
                self.publish_virleader_pos()
                self.publish_formation()
                self.publish_spawn_offset()
                self.publish_adjacency()

            # c2 link hijack attack
            if self.wpt_idx >= 4 and self.wpt_idx < 13:
                self.attack_timer   =   np.clip(self.attack_timer+self.timer_period/self.attack_duration,0,1)
                print('C2 Link Hijacking')

            else:
                self.attack_timer   =   np.float64(0.0)

            if np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_set_pt_ned) < self.nav_wpt_reach_thres and self.wpt_idx < np.shape(self.wpts_ned)[0]-1:
                self.wpt_change_flag    =   True
            
            # exit:
            elif np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_set_pt_ned) < self.nav_wpt_reach_thres and self.wpt_idx == np.shape(self.wpts_ned)[0]-1:
                self.next_phase_flag    =   True


        elif self.flight_phase == 2:
                
            # entry:
            for idx in (idx for idx in range(self.n_drone) if self.entry_execute[idx] is False):

                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,idx,1.0,4.0,5.0)
                self.entry_execute[idx]     =   True

        if self.wpt_change_flag:

            self.omega                  =   np.float64(0.0)
            self.wpt_idx                =   self.wpt_idx+1
            self.vleader_prev_wpt_ned   =   np.copy(self.vleader_next_wpt_ned)
            self.vleader_next_wpt_ned   =   np.copy(self.wpts_ned[self.wpt_idx])
            self.wpt_change_flag        =   False

            print('Waypoint Update: '+str(self.wpt_idx))


        if self.next_phase_flag:

            past_flag_temp          =   self.flight_phase
            self.flight_phase       =   self.flight_phase+1
            new_flag_temp           =   self.flight_phase
            self.next_phase_flag    =   False
            self.entry_execute      =   [False for _ in range(self.n_drone)]

            print('Next Flight Phase %d -> %d' %(past_flag_temp,new_flag_temp))

def main():

    # to be updated: get argument as a separate note for dynamically changing network (ndrones, ref_lla, wpts, formation)
    # parser = argparse.ArgumentParser()
    # parser.add_argument('-n', type=int)
    # args = parser.parse_args()

    n_drone     =   7
    ref_lla     =   np.array([24.484043629238872, 54.36068616768677, 0], dtype=np.float64)    # (lat,lon,alt) -> (deg,deg,m)
    wpts        =   np.array([[24.484043629238872,54.36068616768677,40],
                              [24.484326113268185,54.360644616972564,40],
                              [24.48476311664666,54.3614948536716,40],
                              [24.485097533474377,54.36197496905472,40],
                              [24.485483645928923, 54.36255089202906, 40],
                              [24.485874392399662, 54.363210517962706,40],
                              [24.486215998429138, 54.36370768551379,40],
                              [24.486583103598594, 54.3642176758697,40]], dtype=np.float64)
    
    wpts        =   np.insert(wpts,8,np.flipud(np.copy(wpts)),axis=0)
    wpts        =   np.delete(wpts,8,0)

    # formation   =   np.array([[3.0*np.cos(np.pi/180*0),3.0*np.sin(np.pi/180*0),0],
    #                           [3.0*np.cos(np.pi/180*60),3.0*np.sin(np.pi/180*60),0],
    #                           [3.0*np.cos(np.pi/180*120),3.0*np.sin(np.pi/180*120),0],
    #                           [3.0*np.cos(np.pi/180*180),3.0*np.sin(np.pi/180*180),0],
    #                           [3.0*np.cos(np.pi/180*240),3.0*np.sin(np.pi/180*240),0],
    #                           [3.0*np.cos(np.pi/180*300),3.0*np.sin(np.pi/180*300),0]], dtype=np.float64)

    # formation   =   np.array([[4.0, 0.0, 2.0],
    #                           [-4.0, 0.0, 1.0],
    #                           [2.0, 2.0, -0.5],
    #                           [2.0, -2.0, -0.5],
    #                           [-2.0, -2.0, 0.5],
    #                           [-2.0, 2.0, 0.5]], dtype=np.float64)
    
    formation   =   np.array([[4.0*np.cos(np.pi/180*0),4.0*np.sin(np.pi/180*0),-2.0],
                              [4.0*np.cos(np.pi/180*60),4.0*np.sin(np.pi/180*60),0.0],
                              [4.0*np.cos(np.pi/180*120),4.0*np.sin(np.pi/180*120),-2.0],
                              [4.0*np.cos(np.pi/180*180),4.0*np.sin(np.pi/180*180),0.0],
                              [4.0*np.cos(np.pi/180*240),4.0*np.sin(np.pi/180*240),-2.0],
                              [4.0*np.cos(np.pi/180*300),4.0*np.sin(np.pi/180*300),0.0],
                              [0.0, 0.0, -1.0]], dtype=np.float64)

    adjacency   =   np.array([[0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                              [1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                              [1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                              [1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0],
                              [1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0],
                              [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0],
                              [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]], dtype=np.float64)

    rclpy.init(args=None)

    offboard_mission = OffboardMission(n_drone,wpts,formation,adjacency,ref_lla)

    rclpy.spin(offboard_mission)

    offboard_mission.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()