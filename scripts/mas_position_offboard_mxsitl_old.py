#!/usr/bin/env python3

__author__ = "Minhyun Cho, Yifan Guo, Vishnu Vijay"
__contact__ = "@purdue.edu"

# python packages and modules
import argparse
import numpy as np
from functools import partial
from copy import deepcopy

# ros packages
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# px4/navigation packages
import navpy

# crazyflie packages
# enable for experiment
# from qfly import Pose, QualisysCrazyflie, World, utils, ParallelContexts

# messages
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleGlobalPosition, VehicleCommand
from geometry_msgs.msg import PointStamped
from std_msgs.msg import UInt8, Bool, Float32MultiArray

class OffboardMission(Node):

    def __init__(self,debug,waypoints,formation_config,adjacency_matrix,ref_lla):

        # inheritance from parent class
        super().__init__("mas_position_offboard_mxsitl")

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

        # check debug mode
        if debug:
            self.model_ns   =   ['px4_1', 'px4_2', 'px4_3', 'px4_4', 'px4_5', 'px4_6', 'cf_1', 'cf_2', 'cf_3', 'cf_4']
        else:
            # declare and get the namespace from the launch file
            self.declare_parameter('agent_ns',rclpy.Parameter.Type.STRING_ARRAY)
            self.model_ns       =   self.get_parameter('agent_ns').value

        # define number of drones (both px4 and cf)
        self.n_drone        =   len(self.model_ns)
        self.n_px4          =   0
        self.n_cf           =   0
        
        for ns in self.model_ns:
            if 'px4' in ns:
                self.n_px4  += 1
            if 'cf' in ns:
                self.n_cf   += 1

        # get formation configuration and adjacency matrix of the vehicle network
        if debug:
            self.formation  =   [
                                '2.0*np.cos(np.pi/180*60)','2.0*np.sin(np.pi/180*60)','0.0',     # px4_1
                                '2.0*np.cos(np.pi/180*180)','2.0*np.sin(np.pi/180*180)','0.0',   # px4_2
                                '2.0*np.cos(np.pi/180*300)','2.0*np.sin(np.pi/180*300)','0.0',   # px4_3, attacked drone
                                '2.0*np.cos(np.pi/180*0)','2.0*np.sin(np.pi/180*0)','-0.25',     # px4_4
                                '2.0*np.cos(np.pi/180*120)','2.0*np.sin(np.pi/180*120)','-0.25', # px4_5
                                '2.0*np.cos(np.pi/180*240)','2.0*np.sin(np.pi/180*240)','-0.25', # px4_6
                                '1.0*np.cos(np.pi/180*0)','1.0*np.sin(np.pi/180*0)','0.0',       # cf_1, attacked drone
                                '1.0*np.cos(np.pi/180*120)','1.0*np.sin(np.pi/180*120)','0.0',   # cf_2
                                '1.0*np.cos(np.pi/180*240)','1.0*np.sin(np.pi/180*240)','0.0',   # cf_3
                                '0.0','0.0','0.1'                                                # cf_4 
                                ]
        else:
            self.declare_parameter('formation',rclpy.Parameter.Type.STRING_ARRAY)
            self.formation      =   deepcopy(self.get_parameter('formation').value)

        for idx, x in enumerate(self.formation):
            if isinstance(x, str):
                self.formation[idx]	=   np.float64(eval(x))

            elif isinstance(x, float) or isinstance(x, int):
                self.formation[idx]    =   np.float64(self.formation[idx])

        self.formation  =   np.asarray(self.formation)
        self.formation  =   self.formation.reshape((-1,self.n_drone,3))
        self.formation  =   np.swapaxes(np.swapaxes(self.formation,0,1),1,2)
        for idx in range(self.formation.shape[2]):
            self.get_logger().info(f'{self.formation[:,:,idx]}')
        self.get_logger().info(f'{np.array(self.formation.shape)}')

        if debug:
            self.adjacency  =   [
                                '0','1','1','1','1','1','1','1','1','1',  # px4_1
                                '1','0','1','1','1','1','1','1','1','1',  # px4_2
                                '1','1','0','1','1','1','1','1','1','1',  # px4_3, attacked drone
                                '1','1','1','0','1','1','1','1','1','1',  # px4_4
                                '1','1','1','1','0','1','1','1','1','1',  # px4_5
                                '1','1','1','1','1','0','1','1','1','1',  # px4_6
                                '1','1','1','1','1','1','0','1','1','1',  # cf_1, attacked drone
                                '1','1','1','1','1','1','1','0','1','1',  # cf_2
                                '1','1','1','1','1','1','1','1','0','1',  # cf_3
                                '1','1','1','1','1','1','1','1','1','0'   # cf_4
                                ]
        else:
            self.declare_parameter('adjacency',rclpy.Parameter.Type.STRING_ARRAY)
            self.adjacency      =   deepcopy(self.get_parameter('adjacency').value)

        for idx, x in enumerate(self.adjacency):
            if isinstance(x, str):
                self.adjacency[idx]	=   np.float64(eval(x))

            elif isinstance(x, float) or isinstance(x, int):
                self.adjacency[idx]    =   np.float64(self.adjacency[idx])

        self.adjacency  =   np.asarray(self.adjacency)
        self.adjacency  =   self.adjacency.reshape((self.n_drone,self.n_drone))
        self.get_logger().info(f'{self.adjacency}')

        # set crazyflies body names, comm address, marker ids, ips, world and array publisher
        # self.cf_body_names  =   ['cf1','cf2','cf3','cf4']           # [-] QTM (Qualysis track manager) rigid body name / ['cf1','cf2','cf3','cf4']

        # self.cf_uris        =   ['radio://0/80/2M/E7E7E7E701',
        #                          'radio://0/80/2M/E7E7E7E702',
        #                          'radio://1/80/2M/E7E7E7E703',
        #                          'radio://1/80/2M/E7E7E7E704']      # [-] crazyflie address  
        
        # self.cf_marker_ids  =   [[11, 12, 13, 14],
        #                          [21, 22, 23, 24],
        #                          [31, 32, 33, 34],
        #                          [41, 42, 43, 44]]                  # [-] active marker IDs
        
        # self.cf_body_names  =   ['cf1','cf2']                             # [-] QTM (Qualysis track manager) rigid body name / ['cf1','cf2','cf3','cf4']

        # self.cf_uris        =   ['radio://0/80/2M/E7E7E7E701',
        #                          'radio://0/80/2M/E7E7E7E702']      # [-] crazyflie address  
        
        # self.cf_marker_ids  =   [[11, 12, 13, 14],
        #                          [21, 22, 23, 24]]                  # [-] active marker IDs

        # define subscribers and publishers 
        # px4: all
        # cf: trajectory_pub, spawn_offset_pub, no subscribers
        # common: vleader_pos_pub, formation_pub, adjacency_pub
        self.array_publishers     =   [{'offboard_mode_pub':None, 'trajectory_pub':None, 'vehicle_command_pub':None, 'spawn_offset_pub':None} for _ in range(self.n_drone)]
        self.array_subscribers    =   [{'status_sub':None, 'local_pos_sub':None, 'global_pos_sub':None} for _ in range(self.n_px4)]
        self.element_publisher    =   [{'vleader_pos_pub':None, 'formation_pub':None, 'adjacency_pub':None}]

        for idx in range(self.n_drone):

            if 'px4' in self.model_ns[idx]:

                self.array_subscribers[idx]['status_sub']           =   self.create_subscription(
                    VehicleStatus,
                    f'{self.model_ns[idx]}/fmu/out/vehicle_status',
                    partial(self.vehicle_status_callback,id=idx),                           # instead of lambda function, lambda msg: self.vehicle_status_callback(msg,id=i), use partial function
                    qos_profile_sub)

                self.array_subscribers[idx]['local_pos_sub']        =   self.create_subscription(
                    VehicleLocalPosition,
                    f'{self.model_ns[idx]}/fmu/out/vehicle_local_position',
                    partial(self.local_position_callback,id=idx),
                    qos_profile_sub)

                self.array_subscribers[idx]['global_pos_sub']       =   self.create_subscription(
                    VehicleGlobalPosition,
                    f'{self.model_ns[idx]}/fmu/out/vehicle_global_position',
                    partial(self.global_position_callback,id=idx),
                    qos_profile_sub)

                self.array_publishers[idx]['offboard_mode_pub']     =   self.create_publisher(
                    OffboardControlMode,
                    f'{self.model_ns[idx]}/fmu/in/offboard_control_mode',
                    qos_profile_pub)

                self.array_publishers[idx]['vehicle_command_pub']   =   self.create_publisher(
                    VehicleCommand,
                    f'{self.model_ns[idx]}/fmu/in/vehicle_command',
                    qos_profile_pub)

            self.array_publishers[idx]['trajectory_pub']        =   self.create_publisher(
                TrajectorySetpoint,
                f'{self.model_ns[idx]}/fmu/in/trajectory_setpoint',
                qos_profile_pub)

            self.array_publishers[idx]['spawn_offset_pub']      =   self.create_publisher(
                PointStamped,
                f'{self.model_ns[idx]}/detector/spawn_offset',
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

        # set the reference as ned origin / convert global coordinates to local ones
        self.ref_lla        =   ref_lla
        self.wpt_idx        =   0

        self.wpts_lla       =   waypoints
        self.wpts_ned       =   navpy.lla2ned(self.wpts_lla[:,0], self.wpts_lla[:,1], self.wpts_lla[:,2],
                                              self.ref_lla[0], self.ref_lla[1], self.ref_lla[2],
                                              latlon_unit='deg', alt_unit='m', model='wgs84')
        if len(self.wpts_ned.shape) == 1:
            self.wpts_ned   =   np.reshape(self.wpts_ned, (1,self.wpts_ned.shape[0]))

        # add experimental offset
        self.experimental_offset    =   np.array([0.0,0.5,0.0],dtype=np.float64)
        self.wpts_ned[:,1]          =   self.wpts_ned[:,1]+self.experimental_offset[1]

        # parameters for formation flight
        self.wpt_idx                =   np.uint8(0)
        
        self.velocity               =   np.float64(0.1, dtype=np.float64)           # [m/s] reference velocity of the vehicle
        self.norm                   =   np.float64(0.0, dtype=np.float64)           # [m] distance between each waypoint
        self.dir_vector             =   np.array([0.0,0.0,0.0], dtype=np.float64)   # [-] unit direction vector
        self.look_ahead             =   np.float64(0.0, dtype=np.float64)           # [m] look ahead distance
        self.nav_wpt_reach_thres    =   2.0*self.velocity                           # [m] waypoint reach condition radius

        self.form_idx               =   np.uint8(0)

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
        self.omega_t            =   np.float64(0.0)                 # [-] trajectory interpolation
        self.omega_f            =   np.float64(0.0)                 # [-] formation interpolation

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

        # self.attack_vector[2]               =   0.5*(self.formation[0,:,0]-self.formation[2,:,0])
        # self.attack_vector[2][2]            =   0.5
        # self.attack_vector[6]               =   -0.5*self.formation[6,:,0]

        self.attack_start       =   np.float64(10.0)
        self.attack_duration    =   np.float64(30.0)
        self.attack_timer       =   np.float64(0.0)
        self.attack_engage      =   np.float64(0.0)
        self.attack_speed       =   np.float64(1/10)                    # [sec^(-1)] attack speed

        # enable for experiment
        # self.qtm_ip         =   '192.168.123.2'         # [-] ip setup
        # self.world          =   World(expanse=3.0)      # [-] set up world - the world object comes with sane defaults
        # self.qcfs_          =   [QualisysCrazyflie(cf_body_name,cf_uri,self.world,marker_ids = cf_marker_id,qtm_ip = self.qtm_ip) \
        #                          for cf_body_name, cf_uri, cf_marker_id in zip(self.cf_body_names, self.cf_uris, self.cf_marker_ids)]    # [-] stack up context managers
        # self.qcfs           =   ParallelContexts(*self.qcfs_)
        # self.qcfs           =   self.qcfs.__enter__()

        # parameters for callback
        self.timer_period   =   0.02            # [sec] callback function frequency (offboardcontrolmode should be at least 2Hz)
        self.timer          =   self.create_timer(self.timer_period, self.cmdloop_callback)

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
        if (self.omega_f) < 0.2 and (self.omega_f >= 0) and self.form_idx == 1:
            msg.data            =   (5*self.omega_f*self.formation[:,:,self.form_idx-1]+5*(0.2-self.omega_f)*self.formation[:,:,self.form_idx]-self.ned_spawn_offset).flatten().tolist()
            self.get_logger().info(f'{(5*self.omega_f*self.formation[:,:,self.form_idx-1]+5*(0.2-self.omega_f)*self.formation[:,:,self.form_idx])}')
        else:
            msg.data                =   (self.formation[:,:,self.form_idx]-self.ned_spawn_offset).flatten().tolist()
            self.get_logger().info(f'{(self.formation[:,:,self.form_idx])}')
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
                self.vleader_set_pt_ned     =   np.array([0.0,0.0,-0.5], dtype=np.float64)+self.experimental_offset
                self.vleader_prev_wpt_ned   =   np.array([0.0,0.0,0.0], dtype=np.float64)+self.experimental_offset
                self.vleader_next_wpt_ned   =   np.array([0.0,0.0,-0.5], dtype=np.float64)+self.experimental_offset

            for idx in (idx for idx in range(self.n_drone) if self.entry_execute[idx] is False):
                if idx < self.n_px4:    # px4
                    # compute compensation of local position coordinate
                    if self.global_ref_lla_list[idx] is not None and self.local_pos_ned_list[idx] is not None and self.entry_execute[idx] is False:
                        self.ned_spawn_offset[idx]  =   navpy.lla2ned(self.global_ref_lla_list[idx][0],self.global_ref_lla_list[idx][1],0,
                                                                    self.ref_lla[0],self.ref_lla[1],self.ref_lla[2],
                                                                    latlon_unit='deg', alt_unit='m', model='wgs84')

                        self.trajectory_set_pt[idx] =   np.copy(self.vleader_set_pt_ned)
                        self.yaw_set_pt[idx]        =   self.yaw_set_pt[idx]

                        self.publish_trajectory_setpoint(idx)

                        self.entry_execute[idx]     =   True

                else:                   # cf
                    self.ned_spawn_offset[idx]      =   np.zeros((3,), dtype=np.float64)

                    if idx == self.n_px4:
                        self.trajectory_set_pt[idx] =   np.array([0.5,0.5,-0.5], dtype=np.float64)

                    elif idx == self.n_px4+1:
                        self.trajectory_set_pt[idx] =   np.array([-0.5,0.5,-0.5], dtype=np.float64)

                    elif idx == self.n_px4+2:
                        self.trajectory_set_pt[idx] =   np.array([-1.0,0.5,-0.5], dtype=np.float64)

                    elif idx == self.n_px4+3:
                        self.trajectory_set_pt[idx] =   np.array([0.0,0.5,-0.5], dtype=np.float64)

                    self.yaw_set_pt[idx]        =   self.yaw_set_pt[idx]
                    self.entry_execute[idx]     =   True

            # during:
            for idx in range(self.n_px4):
                if self.nav_state_list[idx] != VehicleStatus.ARMING_STATE_ARMED or self.arm_counter_list[idx] < 20:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,idx,1.0,6.0)
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,idx,1.0)
                    self.arm_counter_list[idx]  +=  1
                    # self.get_logger().info('px4 #'+str(idx+1)+' arming engaged ...')

            for idx in (idx for idx in range(self.n_px4) if self.entry_execute[idx] is True):
                self.publish_offboard_control_mode(idx)
                # self.get_logger().info('px4 #'+str(idx+1)+' takeoff(offboard) engaged ...')

            for idx in (idx for idx in range(self.n_drone) if self.entry_execute[idx] is True):
                self.publish_trajectory_setpoint(idx)

            # enable for experiment
            # if (all(qcf.is_safe() for qcf in self.qcfs)):
            #     # cycle for crazyflies
            #     for idx, qcf in enumerate(self.qcfs): 
            #         qcf.safe_position_setpoint(Pose(self.trajectory_set_pt[idx+self.n_px4][1],self.trajectory_set_pt[idx+self.n_px4][0],-self.trajectory_set_pt[idx+self.n_px4][2]))
            #         qcf.set_led_ring(7)
            #         qcf.cf.param.set_value('ring.solidGreen', 200)
            #         # self.get_logger().info('cf #'+str(idx+1)+' takeoff engaged ...')

            # else:
            #     # land
            #     for idx, qcf in enumerate(self.qcfs):
            #         qcf.land_in_place()
            #         # self.get_logger().info('cf #'+str(idx+1)+' cannot takeoff and land ...')

            # exit:
            if [True for idx in range(self.n_px4) if (self.local_pos_ned_list[idx] is not None)] == [True for idx in range(self.n_px4)] and \
                [True for idx in range(self.n_cf) if (self.entry_execute[idx+self.n_px4])] == [True for idx in range(self.n_cf)]:
                if [True for idx in range(self.n_px4) if (np.linalg.norm(self.trajectory_set_pt[idx]-self.local_pos_ned_list[idx]) < self.nav_wpt_reach_thres)] == \
                    [True for idx in range(self.n_px4)]:
                    self.next_phase_flag        =   True 


        elif self.flight_phase == 1:
                
            # entry:
            if all(not agent_entry for agent_entry in self.entry_execute):
                self.vleader_prev_wpt_ned   =   np.copy(self.vleader_next_wpt_ned)
                self.vleader_next_wpt_ned   =   np.copy(self.wpts_ned[self.wpt_idx])
                self.omega_t                =   np.float64(0.0)

                self.norm                   =   np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)
                self.normvector             =   (self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)/self.norm

                for axis in range(3):
                    self.vleader_set_pt_ned[axis]   =   np.clip((1-self.omega_t)*self.vleader_prev_wpt_ned[axis] \
                                                                +self.omega_t*self.vleader_next_wpt_ned[axis], \
                                                                np.min(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])), \
                                                                np.max(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])))    

            # during:
            else:
                norm_delt       =   np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)/self.velocity
                form_delt       =   10

                self.omega_t    =   self.omega_t+self.timer_period/norm_delt
                self.omega_t    =   np.clip(self.omega_t,0,1)

                if (self.form_idx < self.formation.shape[2]-1) and self.omega_f == 1:
                    self.form_idx   +=  1
                    self.omega_f    =   0
                self.omega_f    =   self.omega_f+self.timer_period/form_delt
                self.omega_f    =   np.clip(self.omega_f,0,1)

                self.norm                   =   np.linalg.norm(self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)
                self.normvector             =   (self.vleader_next_wpt_ned-self.vleader_prev_wpt_ned)/self.norm

                for axis in range(3):
                    self.vleader_set_pt_ned[axis]   =   np.clip((1-self.omega_t)*self.vleader_prev_wpt_ned[axis] \
                                                                +self.omega_t*self.vleader_next_wpt_ned[axis], \
                                                                np.min(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])), \
                                                                np.max(np.array([self.vleader_prev_wpt_ned[axis], self.vleader_next_wpt_ned[axis]])))    

            # entry/during:                
            for idx in range(self.n_drone):
                self.trajectory_set_pt[idx]     =   np.copy(self.vleader_set_pt_ned)
                self.trajectory_set_pt[idx]     =   self.trajectory_set_pt[idx]+(self.formation[idx,:,self.form_idx]-self.ned_spawn_offset[idx]) \
                                                    +self.look_ahead*self.normvector+self.attack_vector[idx]*self.attack_engage             # [m] Disable look-ahead
                self.yaw_set_pt[idx]            =   self.yaw_set_pt[idx]

                self.publish_trajectory_setpoint(idx)

                if self.entry_execute[idx] is False:
                    self.entry_execute[idx]     =   True

            # cycle for px4
            for idx in range(self.n_px4):
                self.publish_offboard_control_mode(idx)

            # enable for experiment
            # if (all(qcf.is_safe() for qcf in self.qcfs)):
            #     # cycle for crazyflies
            #     for idx, qcf in enumerate(self.qcfs): 
            #         qcf.safe_position_setpoint(Pose(self.trajectory_set_pt[idx+self.n_px4][1],self.trajectory_set_pt[idx+self.n_px4][0],-self.trajectory_set_pt[idx+self.n_px4][2]))
                    
            #         if (idx == 0) and (self.attack_timer >= self.attack_start):
            #             qcf.set_led_ring(7)
            #             qcf.cf.param.set_value('ring.solidGreen', 0)
            #             if np.round(self.attack_timer)%2 == 1:
            #                 # self.get_logger().info('Blinking ...'+str(np.round(self.attack_timer)%2))
            #                 qcf.cf.param.set_value('ring.solidRed', 200)
            #             else:
            #                 # self.get_logger().info('Blinking ...'+str(np.round(self.attack_timer)%2))
            #                 qcf.cf.param.set_value('ring.solidRed', 50)
            #         else:
            #             qcf.set_led_ring(7)
            #             qcf.cf.param.set_value('ring.solidGreen', 200)

            # else:
            #     # land
            #     for idx, qcf in enumerate(self.qcfs):
            #         qcf.land_in_place()

            # publish the information for fdir node
            if (self.wpt_idx <= np.shape(self.wpts_ned)[0]-1): # and (self.omega_t == 1.0):
                self.publish_virleader_pos()
                self.publish_spawn_offset()
            self.publish_formation()
            self.publish_adjacency()
                
            # else:
                # self.get_logger().info('Drones approaching the target position ...')

            # c2 link hijack attack engaging (for experiment):
            if (self.wpt_idx == np.shape(self.wpts_ned)[0]-1) and (self.omega_t == 1.0):
                self.attack_timer      =   self.attack_timer+self.timer_period

                if (self.attack_timer >= self.attack_start) and (self.attack_timer < self.attack_duration+self.attack_start):
                    self.attack_engage      =   np.clip(self.attack_engage+self.timer_period*self.attack_speed,0,1)
                    # self.get_logger().info('Drones under the attack ...'+str(self.attack_engage))

                else:
                    self.attack_engage      =   np.float64(0.0)
                    # self.get_logger().info('Preparing for the attack ...')

            if (self.wpt_idx < np.shape(self.wpts_ned)[0]-1) and self.omega_t >= 1.0:
                self.wpt_change_flag    =   True
            
            # exit:
            elif (self.wpt_idx == np.shape(self.wpts_ned)[0]-1) and (self.attack_timer > self.attack_duration+self.attack_start):
                self.next_phase_flag    =   True


        elif self.flight_phase == 2:
                
            # entry:
            for idx in (idx for idx in range(self.n_px4) if self.entry_execute[idx] is False):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,idx,1.0,4.0,5.0)
                self.entry_execute[idx]     =   True

            # enable for experiment
            # for idx, qcf in enumerate(self.qcfs):
            #     qcf.land_in_place()
            #     self.entry_execute[idx+self.n_px4]  =   True

        if self.wpt_change_flag:

            self.omega_t                =   np.float64(0.0)
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

    debug       =   False
    ref_lla     =   np.array([24.484043629238872,54.36068616768677,0], dtype=np.float64)    # (lat,lon,alt) -> (deg,deg,m)

    wpts_ned    =   np.array([[0.0,0.0,0.5],[0.01,0.0,0.5]],dtype=np.float64) #                              [3,0.0,0.5],[3,3,0.5]
    
    wpts_temp   =   navpy.ned2lla(wpts_ned,ref_lla[0],ref_lla[1],ref_lla[2],latlon_unit='deg',alt_unit='m',model='wgs84')
    wpts        =   np.concatenate((wpts_temp[0][:,np.newaxis],wpts_temp[1][:,np.newaxis],-wpts_temp[2][:,np.newaxis]),axis=1)

    formation   =   np.array([[2.0*np.cos(np.pi/180*60),2.0*np.sin(np.pi/180*60),0.0],        # px4_1
                              [2.0*np.cos(np.pi/180*180),2.0*np.sin(np.pi/180*180),0.0],      # px4_2
                              [2.0*np.cos(np.pi/180*300),2.0*np.sin(np.pi/180*300),0.0],      # px4_3, attacked drone
                              [1.0*np.cos(np.pi/180*0),1.0*np.sin(np.pi/180*0),0.0],           # cf_1, attacked drone
                              [1.0*np.cos(np.pi/180*120),1.0*np.sin(np.pi/180*120),0.0],       # cf_2
                              [1.0*np.cos(np.pi/180*240),1.0*np.sin(np.pi/180*240),0.0],       # cf_3
                              [0.0, 0.0, 0.1]], dtype=np.float64)                              # cf_4

    adjacency   =   np.array([[0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                              [1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                              [1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                              [1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0],
                              [1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0],
                              [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0],
                              [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]], dtype=np.float64)

    rclpy.init(args=None)

    offboard_mission = OffboardMission(debug,wpts,formation,adjacency,ref_lla)

    rclpy.spin(offboard_mission)

    offboard_mission.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()