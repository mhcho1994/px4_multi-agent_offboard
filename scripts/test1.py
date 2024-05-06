        # set the reference as ned origin / convert global coordinates to local ones
        self.ref_lla        =   ref_lla
        self.wpt_idx        =   0

        self.wpts_lla       =   waypoints
        self.wpts_ned       =   navpy.lla2ned(self.wpts_lla[:,0], self.wpts_lla[:,1], self.wpts_lla[:,2],
                                              self.ref_lla[0], self.ref_lla[1], self.ref_lla[2],
                                              latlon_unit='deg', alt_unit='m', model='wgs84')

        # parameters for formation flight
        self.wpt_idx        =   np.uint8(0)
        
        self.velocity       =   np.float64(4.0)         # [m/s] reference velocity of the vehicle
        self.nav_wpt_reach_rad  =   1.0*self.velocity   # [m] waypoint reach condition radius

        # variables for virtual leader
        self.vleader_set_pt_ned     =   np.array([0.0,0.0,0.0], dtype=np.float64)
        self.vleader_prev_wpt_ned   =   np.array([0.0,0.0,0.0], dtype=np.float64)
        self.vleader_next_wpt_ned   =   np.array([0.0,0.0,0.0], dtype=np.float64)

        # variables for agents
        self.entry_execute      =   [False for _ in range(self.n_drone)]
        self.ned_spawn_offset   =   [None for _ in range(self.n_drone)]
        self.flight_phase       =   np.uint8(0)
        self.nav_state_list     =   [VehicleStatus.NAVIGATION_STATE_MAX for _ in range(self.n_drone)]
        self.next_phase_flag    =   [False for _ in range(self.n_drone)]
        self.wpt_change_flag    =   [False for _ in range(self.n_drone)]
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

        self.attack_vector[4]   =   self.formation[3,:]-self.formation[4,:]
        self.attack_vector[2]   =   3*self.formation[2,:]

        self.attack_duration    =   np.float64(10.0)
        self.attack_timer       =   np.float64(0.0)