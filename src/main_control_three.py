#!/usr/bin/env python3

# package
import os
import rospy
import yaml
import smach
import numpy as np
import cv2
import threading

# message
from std_msgs.msg import Empty
from load_transport.msg import position_msg, state_machine_msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix

# Hardward Config
from hardware import Payload, Drone

# util
from util_PID_control import PID, PID_z, PID_single_var

class sWarmup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['warmup_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WARMUP')

        rospy.sleep(5.0)
        pub1.util_motor_on()
        pub2.util_motor_on()
        pub3.util_motor_on()
        rospy.sleep(5.0)
        pub_sm.util_smach('WARM_UP', 'FLYUP_OPEN')
        return 'warmup_finish'

class sFlyupOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_open_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FLYUP_OPEN')
        
        thrust_01 = float(rospy.get_param('~lift_thrust_01', "1.5"))
        thrust_02 = float(rospy.get_param('~lift_thrust_02', "1.5"))
        thrust_03 = float(rospy.get_param('~lift_thrust_03', "1.5"))
        duration = float(rospy.get_param('~lift_duration', "3"))

        pub1.util_cmd(0, 0, thrust_01, 0)
        pub2.util_cmd(0, 0, thrust_02, 0)
        pub3.util_cmd(0, 0, thrust_03, 0)

        rospy.sleep(duration)

        pub_sm.util_smach('FLYUP_OPEN', 'HEIGHT_CONTROL')
        return 'flyup_open_finish'

class sTakeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeoff_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TAKEOFF')
        
        pub1.util_takeoff()
        pub2.util_takeoff()
        pub3.util_takeoff()

        rospy.sleep(7.0)

        pub_sm.util_smach('TAKEOFF', 'YAW_SEARCH')
        return 'takeoff_finish'

class sHover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hover_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HOVER')

        pub1.util_hover()
        pub2.util_hover()
        pub3.util_hover()

        rospy.sleep(15.0)

        pub_sm.util_smach('HOVER', 'LEFT')
        return 'hover_finish'

class sLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['left_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LEFT')

        pub1.util_cmd(-0.5, 0, 0, 0)
        pub2.util_cmd(-0.5, 0, 0, 0)
        pub3.util_cmd(-0.5, 0, 0, 0)

        rospy.sleep(10.0)

        pub_sm.util_smach('LEFT', 'RIGHT')
        return 'left_finish'

class sRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['right_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RIGHT')

        pub1.util_cmd(0.5, 0, 0, 0)
        pub2.util_cmd(0.5, 0, 0, 0)
        pub3.util_cmd(0.5, 0, 0, 0)

        rospy.sleep(10.0)

        pub_sm.util_smach('RIGHT', 'LAND')
        return 'right_finish'

class sYawSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ys_finish'])

        # load pid gain
        kp_pitch = float(rospy.get_param('~pitch_kp', "0.025"))
        ki_pitch = float(rospy.get_param('~pitch_ki', "0.025"))
        kd_pitch = float(rospy.get_param('~pitch_kd', "0.025"))
        self.pid_pitch_1 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_2 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_3 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)

        kp_yaw = float(rospy.get_param('~yaw_kp', "0.025"))
        ki_yaw = float(rospy.get_param('~yaw_ki', "0.025"))
        kd_yaw = float(rospy.get_param('~yaw_kd', "0.025"))
        self.pid_yaw_1 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_2 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_3 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)

        # set target
        # height_d = float(rospy.get_param('~height_d', "1.2"))
        # yaw_tol = float(rospy.get_param('~yaw_tol', "5"))

        self.pid_yaw_1.setTarget(0)
        self.pid_pitch_1.setTarget(-3*np.pi/180)
        self.pid_yaw_2.setTarget(0)
        self.pid_pitch_2.setTarget(-3*np.pi/180)
        self.pid_yaw_3.setTarget(0)
        self.pid_pitch_3.setTarget(-3*np.pi/180)
        # self.pid1r.setTarget(0, yaw_tol)
        # self.pid2r.setTarget(0, yaw_tol)
        # self.pid3r.setTarget(0, yaw_tol)

    def execute(self, userdata):
        rospy.loginfo('Executing state YAW SEARCH')   

        sub1.bl = None
        sub1.nbl = None
        sub2.bl = None
        sub2.nbl = None
        sub3.bl = None
        sub3.nbl = None
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            if ((sub1.bl is not None) and (sub2.bl is not None) and (sub3.bl is not None)):
                if ((self.pid_yaw_1.err < 0.09) and (self.pid_yaw_2.err < 0.09) and (self.pid_yaw_3.err < 0.09)):
                    break

            if sub1.bl is None:
                u1z = 0.3
                # u1z = 0.1
                u1r = 0.4
            else:
                index = 1
                psi = self.yawError(index)
                u1r = self.pid_yaw_1.update(psi)         # desired: 0

                phi = self.pitchError(index)
                u1z = self.pid_pitch_1.update(phi)         # desired: 0

            if sub2.bl is None:
                u2z = 0
                # u2z = 0.1
                u2r = 0.4
            else:
                index = 2
                psi = self.yawError(index)
                u2r = self.pid_yaw_2.update(psi)         # desired: 0

                phi = self.pitchError(index)
                u2z = self.pid_pitch_2.update(phi)         # desired: 0

            if sub3.bl is None:
                u3z = 0
                # u3z = 0.1
                u3r = 0.4
            else:
                index = 3
                psi = self.yawError(index)
                u3r = self.pid_yaw_3.update(psi)         # desired: 0

                phi = self.pitchError(index)
                u3z = self.pid_pitch_3.update(phi)         # desired: 0


            pub1.util_yaw_error(self.pid_yaw_1.err)
            pub1.util_pitch_error(self.pid_pitch_1.err)
            pub2.util_yaw_error(self.pid_yaw_2.err)
            pub2.util_pitch_error(self.pid_pitch_2.err)
            pub3.util_yaw_error(self.pid_yaw_3.err)
            pub3.util_pitch_error(self.pid_pitch_3.err)
            pub1.util_cmd(0, 0, u1z, u1r)
            pub2.util_cmd(0, 0, u2z, u2r)
            pub3.util_cmd(0, 0, u3z, u3r)
            rate.sleep()
        
        pub_sm.util_smach('YAW_SEARCH', 'FORMATION_CONTROL')
        return 'ys_finish'

    def yawError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        nbl = np.copy(sub_.nbl)
        nbl[1] = 0
        psi = nbl@np.array([0,0,1])
        psi = psi / np.sqrt(nbl@nbl)
        psi = np.arccos(psi)
        psi = -np.sign(nbl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi

        return psi

    def pitchError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        be = (Drone.camTilt)@sub_.nbl
        be[0] = 0
        phi = be@np.array([0,0,1])
        phi = phi / np.sqrt(be@be)
        phi = np.arccos(phi)
        phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
        return phi

class sBearingStabilization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fc_finish'])

        # load pid gain
        kp_pitch = float(rospy.get_param('~pitch_kp', "0.025"))
        ki_pitch = float(rospy.get_param('~pitch_ki', "0.025"))
        kd_pitch = float(rospy.get_param('~pitch_kd', "0.025"))
        self.pid_pitch_1 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_2 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_3 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)

        kp_yaw = float(rospy.get_param('~yaw_kp', "0.025"))
        ki_yaw = float(rospy.get_param('~yaw_ki', "0.025"))
        kd_yaw = float(rospy.get_param('~yaw_kd', "0.025"))
        self.pid_yaw_1 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_2 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_3 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)

        self.k_form = float(rospy.get_param('~form_k', "0.025"))

        # set target
        self.pid_yaw_1.setTarget(0)
        self.pid_pitch_1.setTarget(-3*np.pi/180)
        self.pid_yaw_2.setTarget(0)
        self.pid_pitch_2.setTarget(-3*np.pi/180)
        self.pid_yaw_3.setTarget(0)
        self.pid_pitch_3.setTarget(-3*np.pi/180)

        self.t12 = np.array([1,0,0])
        self.t23 = np.array([-1/2,np.sqrt(3)/2,0])
        self.t31 = np.array([-1/2,-np.sqrt(3)/2,0])

    def execute(self, userdata):
        rospy.loginfo('Executing state FORMATION CONTROL')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():           
            # yaw pitch control
            index = 1
            psi = self.yawError(index)
            u1r = self.pid_yaw_1.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u1z = self.pid_pitch_1.update(phi)         # desired: 0
            pub1.util_yaw_error(self.pid_yaw_1.err)
            pub1.util_pitch_error(self.pid_pitch_1.err)

            index = 2
            psi = self.yawError(index)
            u2r = self.pid_yaw_2.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u2z = self.pid_pitch_2.update(phi)         # desired: 0
            pub2.util_yaw_error(self.pid_yaw_2.err)
            pub2.util_pitch_error(self.pid_pitch_2.err)

            index = 3
            psi = self.yawError(index)
            u3r = self.pid_yaw_3.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u3z = self.pid_pitch_3.update(phi)         # desired: 0
            pub3.util_yaw_error(self.pid_yaw_3.err)
            pub3.util_pitch_error(self.pid_pitch_3.err)

            # formation control
            index = 1
            bg1 = self.bgProject2xy(index)
            index = 2
            bg2 = self.bgProject2xy(index)
            index = 3
            bg3 = self.bgProject2xy(index)
            
            pub1.util_bearing_error(np.arccos(bg1@self.t12))
            pub2.util_bearing_error(np.arccos(bg2@self.t23))
            pub3.util_bearing_error(np.arccos(bg3@self.t31))

            # if ((err1 < 0.08) and (err2 < 0.08) and (err3 < 0.08)):
            #     break
            u1 = self.k_form*(bg1 - self.t12 - bg3 + self.t31 )
            u2 = self.k_form*(bg2 - self.t23 - bg1 + self.t12)
            u3 = self.k_form*(bg3 - self.t31 - bg2 + self.t23)
            u1 = (sub1.iRb.T)@u1
            u2 = (sub2.iRb.T)@u2
            u3 = (sub3.iRb.T)@u3

            pub1.util_cmd(u1[0], u1[1], u1z, u1r)
            pub2.util_cmd(u2[0], u2[1], u2z, u2r)
            pub3.util_cmd(u3[0], u3[1], u3z, u3r)
            rate.sleep()
        
        pub_sm.util_smach('FORMATION_CONTROL', 'LAND')
        return 'fc_finish'

    def yawError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        nbl = np.copy(sub_.nbl)
        nbl[1] = 0
        psi = nbl@np.array([0,0,1])
        psi = psi / np.sqrt(nbl@nbl)
        psi = np.arccos(psi)
        psi = -np.sign(nbl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi

        return psi

    def pitchError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        be = (Drone.camTilt)@sub_.nbl
        be[0] = 0
        phi = be@np.array([0,0,1])
        phi = phi / np.sqrt(be@be)
        phi = np.arccos(phi)
        phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
        return phi

    def bgProject2xy(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        bg = np.copy(sub_.bg)
        bg[2] = 0
        return bg / np.sqrt(bg@bg)

class sDistanceLeaderlessStabilization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fc_finish'])

        # load pid gain
        kp_pitch = float(rospy.get_param('~pitch_kp', "0.025"))
        ki_pitch = float(rospy.get_param('~pitch_ki', "0.025"))
        kd_pitch = float(rospy.get_param('~pitch_kd', "0.025"))
        self.pid_pitch_1 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_2 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_3 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)

        kp_yaw = float(rospy.get_param('~yaw_kp', "0.025"))
        ki_yaw = float(rospy.get_param('~yaw_ki', "0.025"))
        kd_yaw = float(rospy.get_param('~yaw_kd', "0.025"))
        self.pid_yaw_1 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_2 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_3 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)

        self.k_form = float(rospy.get_param('~form_k', "0.025"))
        self.k_dist = float(rospy.get_param('~dist_k', "0.025"))

        # set target
        self.pid_yaw_1.setTarget(0)
        self.pid_pitch_1.setTarget(-3*np.pi/180)
        self.pid_yaw_2.setTarget(0)
        self.pid_pitch_2.setTarget(-3*np.pi/180)
        self.pid_yaw_3.setTarget(0)
        self.pid_pitch_3.setTarget(-3*np.pi/180)

        self.t12 = np.array([1,0,0])
        self.t23 = np.array([-1/2,np.sqrt(3)/2,0])
        self.t31 = np.array([-1/2,-np.sqrt(3)/2,0])
        self.l31 = float(rospy.get_param('~desired_dist', "0.025"))

    def execute(self, userdata):
        rospy.loginfo('Executing state FORMATION CONTROL')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():           
            # yaw pitch control
            index = 1
            psi = self.yawError(index)
            u1r = self.pid_yaw_1.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u1z = self.pid_pitch_1.update(phi)         # desired: 0
            pub1.util_yaw_error(self.pid_yaw_1.err)
            pub1.util_pitch_error(self.pid_pitch_1.err)

            index = 2
            psi = self.yawError(index)
            u2r = self.pid_yaw_2.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u2z = self.pid_pitch_2.update(phi)         # desired: 0
            pub2.util_yaw_error(self.pid_yaw_2.err)
            pub2.util_pitch_error(self.pid_pitch_2.err)

            index = 3
            psi = self.yawError(index)
            u3r = self.pid_yaw_3.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u3z = self.pid_pitch_3.update(phi)         # desired: 0
            pub3.util_yaw_error(self.pid_yaw_3.err)
            pub3.util_pitch_error(self.pid_pitch_3.err)

            # formation control
            index = 1
            l12, bg1 = self.bgProject2xy(index)
            index = 2
            l23, bg2 = self.bgProject2xy(index)
            index = 3
            l31, bg3 = self.bgProject2xy(index)
            
            pub1.util_bearing_error(np.arccos(bg1@self.t12))
            pub2.util_bearing_error(np.arccos(bg2@self.t23))
            pub3.util_bearing_error(np.arccos(bg3@self.t31))

            # if ((err1 < 0.08) and (err2 < 0.08) and (err3 < 0.08)):
            #     break
            u1 = self.k_form*(bg1 - self.t12 - bg3 + self.t31) + self.k_dist*(-l31*(self.t31@bg3)*self.t31 + self.l31*self.t31)
            u2 = self.k_form*(bg2 - self.t23 - bg1 + self.t12)
            u3 = self.k_form*(bg3 - self.t31 - bg2 + self.t23) + self.k_dist*(l31*(self.t31@bg3)*self.t31 - self.l31*self.t31)
            u1 = (sub1.iRb.T)@u1
            u2 = (sub2.iRb.T)@u2
            u3 = (sub3.iRb.T)@u3

            pub1.util_cmd(u1[0], u1[1], u1z, u1r)
            pub2.util_cmd(u2[0], u2[1], u2z, u2r)
            pub3.util_cmd(u3[0], u3[1], u3z, u3r)
            rate.sleep()
        
        pub_sm.util_smach('FORMATION_CONTROL', 'LAND')
        return 'fc_finish'

    def yawError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        nbl = np.copy(sub_.nbl)
        nbl[1] = 0
        psi = nbl@np.array([0,0,1])
        psi = psi / np.sqrt(nbl@nbl)
        psi = np.arccos(psi)
        psi = -np.sign(nbl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi

        return psi

    def pitchError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        be = (Drone.camTilt)@sub_.nbl
        be[0] = 0
        phi = be@np.array([0,0,1])
        phi = phi / np.sqrt(be@be)
        phi = np.arccos(phi)
        phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
        return phi

    def bgProject2xy(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        bg = np.copy(sub_.bg)
        bg[2] = 0
        distance = np.sqrt(bg@bg)
        return distance, bg / distance

class sDistanceLeaderStabilization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fc_finish'])

        # load pid gain
        kp_pitch = float(rospy.get_param('~pitch_kp', "0.025"))
        ki_pitch = float(rospy.get_param('~pitch_ki', "0.025"))
        kd_pitch = float(rospy.get_param('~pitch_kd', "0.025"))
        self.pid_pitch_1 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_2 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)
        self.pid_pitch_3 = PID_single_var(kp_pitch, ki_pitch, kd_pitch)

        kp_yaw = float(rospy.get_param('~yaw_kp', "0.025"))
        ki_yaw = float(rospy.get_param('~yaw_ki', "0.025"))
        kd_yaw = float(rospy.get_param('~yaw_kd', "0.025"))
        self.pid_yaw_1 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_2 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)
        self.pid_yaw_3 = PID_single_var(kp_yaw, ki_yaw, kd_yaw)

        self.k_form = float(rospy.get_param('~form_k', "0.025"))
        self.k_dist = float(rospy.get_param('~dist_k', "0.025"))

        # set target
        self.pid_yaw_1.setTarget(0)
        self.pid_pitch_1.setTarget(-3*np.pi/180)
        self.pid_yaw_2.setTarget(0)
        self.pid_pitch_2.setTarget(-3*np.pi/180)
        self.pid_yaw_3.setTarget(0)
        self.pid_pitch_3.setTarget(-3*np.pi/180)

        self.t12 = np.array([1,0,0])
        self.t23 = np.array([-1/2,np.sqrt(3)/2,0])
        self.t31 = np.array([-1/2,-np.sqrt(3)/2,0])
        self.l31 = float(rospy.get_param('~desired_dist', "0.025"))

        # self.first = True

    def execute(self, userdata):
        rospy.loginfo('Executing state FORMATION CONTROL')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():           
            # yaw pitch control
            index = 1
            psi = self.yawError(index)
            u1r = self.pid_yaw_1.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u1z = self.pid_pitch_1.update(phi)         # desired: 0
            pub1.util_yaw_error(self.pid_yaw_1.err)
            pub1.util_pitch_error(self.pid_pitch_1.err)

            index = 2
            psi = self.yawError(index)
            u2r = self.pid_yaw_2.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u2z = self.pid_pitch_2.update(phi)         # desired: 0
            pub2.util_yaw_error(self.pid_yaw_2.err)
            pub2.util_pitch_error(self.pid_pitch_2.err)

            index = 3
            psi = self.yawError(index)
            u3r = self.pid_yaw_3.update(psi)         # desired: 0
            phi = self.pitchError(index)
            u3z = self.pid_pitch_3.update(phi)         # desired: 0
            pub3.util_yaw_error(self.pid_yaw_3.err)
            pub3.util_pitch_error(self.pid_pitch_3.err)

            # formation control
            index = 1
            l12, bg1 = self.bgProject2xy(index)
            index = 2
            l23, bg2 = self.bgProject2xy(index)
            index = 3
            l31, bg3 = self.bgProject2xy(index)
            
            pub1.util_bearing_error(np.arccos(bg1@self.t12))
            pub2.util_bearing_error(np.arccos(bg2@self.t23))
            pub3.util_bearing_error(np.arccos(bg3@self.t31))

            # if ((err1 < 0.08) and (err2 < 0.08) and (err3 < 0.08)):
            #     break
            # if self.first:
            #     print("======================")
            #     print((sub1.iRb.T)@(self.k_form*(bg1 - self.t12 - bg3 + self.t31)))
            #     print((sub1.iRb.T)@(self.k_dist*(-l31*(self.t31@bg3)*self.t31 + self.l31*self.t31)))
            #     self.first = False

            u1 = self.k_form*(bg1 - self.t12 - bg3 + self.t31) + self.k_dist*(-l31*(self.t31@bg3)*self.t31 + self.l31*self.t31)
            u2 = self.k_form*(bg2 - self.t23 - bg1 + self.t12)
            # u3 = self.k_form*(bg3 - self.t31 - bg2 + self.t23) + self.k_dist*(l31*(self.t31@bg3)*self.t31 - self.l31*self.t31)
            u1 = (sub1.iRb.T)@u1
            u2 = (sub2.iRb.T)@u2
            # u3 = (sub3.iRb.T)@u3

            pub1.util_cmd(u1[0], u1[1], u1z, u1r)
            pub2.util_cmd(u2[0], u2[1], u2z, u2r)
            pub3.util_cmd(0, 0, u3z, u3r)
            rate.sleep()
        
        pub_sm.util_smach('FORMATION_CONTROL', 'LAND')
        return 'fc_finish'

    def yawError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        nbl = np.copy(sub_.nbl)
        nbl[1] = 0
        psi = nbl@np.array([0,0,1])
        psi = psi / np.sqrt(nbl@nbl)
        psi = np.arccos(psi)
        psi = -np.sign(nbl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi

        return psi

    def pitchError(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        be = (Drone.camTilt)@sub_.nbl
        be[0] = 0
        phi = be@np.array([0,0,1])
        phi = phi / np.sqrt(be@be)
        phi = np.arccos(phi)
        phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
        return phi

    def bgProject2xy(self, index):
        if index == 1:
            sub_ = sub1
        elif index == 2:
            sub_ = sub2
        elif index == 3:
            sub_ = sub3

        bg = np.copy(sub_.bg)
        bg[2] = 0
        distance = np.sqrt(bg@bg)
        return distance, bg / distance

# class sDistanceLeaderTransport(smach.State):


class sLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['land_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LAND')

        pub1.util_hover()
        pub2.util_hover()
        pub3.util_hover()
        rospy.sleep(0.5)
        pub1.util_land()
        pub2.util_land()
        pub3.util_land()
        return 'land_finish'


class Control():
    def __init__(self):        
        self.sm_top = smach.StateMachine(outcomes=['control_finish'])
        with self.sm_top:
            smach.StateMachine.add('WARMUP', sWarmup(), 
                transitions={'warmup_finish':'FLYUP_OPEN'})
            smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                transitions={'flyup_open_finish':'YAW_SEARCH'})

            # smach.StateMachine.add('YAW_SEARCH', sYawSearch(), 
            #     transitions={'ys_finish':'LAND'})
            smach.StateMachine.add('YAW_SEARCH', sYawSearch(), 
                transitions={'ys_finish':'FORMATION_CONTROL'})
            # smach.StateMachine.add('FORMATION_CONTROL', sBearingStabilization(), 
            #     transitions={'fc_finish':'LAND'})
            # smach.StateMachine.add('FORMATION_CONTROL', sDistanceLeaderlessStabilization(), 
            #     transitions={'fc_finish':'LAND'})
            smach.StateMachine.add('FORMATION_CONTROL', sDistanceLeaderStabilization(), 
                transitions={'fc_finish':'LAND'})

            smach.StateMachine.add('LAND', sLand(), 
                transitions={'land_finish':'control_finish'})
        smach_thread = threading.Thread(target=self.sm_top.execute, daemon = True)
        smach_thread.start()


class Pubs():
    def __init__(self, tello_ns):
        # publisher
        self.pub_motor_on = rospy.Publisher('/%s/manual_takeoff' % tello_ns, Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/%s/takeoff' % tello_ns, Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/%s/cmd_vel' % tello_ns, Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/%s/land' % tello_ns, Empty, queue_size=1)
        self.pub_yaw_error = rospy.Publisher('/%s/yaw/error' % tello_ns, position_msg, queue_size=1)
        self.pub_pitch_error = rospy.Publisher('/%s/pitch/error' % tello_ns, position_msg, queue_size=1)
        self.pub_bearing_error = rospy.Publisher('/%s/bearing/error' % tello_ns, position_msg, queue_size=1)

    def util_cmd(self, x, y, z, yaw):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = yaw
        self.pub_cmd_vel.publish(msg)

    def util_motor_on(self):
        self.pub_motor_on.publish()

    def util_takeoff(self):
        self.pub_takeoff.publish()

    def util_hover(self):
        self.util_cmd(0,0,0,0)

    def util_land(self):
        self.pub_land.publish()

    def util_yaw_error(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_yaw_error.publish(msg)

    def util_pitch_error(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_pitch_error.publish(msg)

    def util_bearing_error(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_bearing_error.publish(msg)

class PubSm():
    def __init__(self):
        self.pub_smach = rospy.Publisher('/state_transition', state_machine_msg, queue_size=1)
    def util_smach(self, state_before, state_after):
        msg = state_machine_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.before = state_before
        msg.after = state_after
        self.pub_smach.publish(msg)

class Subs():
    # subscriber and perception
    def __init__(self, tello_ns):
        self.iRb = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.bl = None
        self.nbl = None
        self.bg = None
        self.nbg = None
        
        self.sub_odom = rospy.Subscriber('/%s/odom' % tello_ns, Odometry, self.cb_odom, queue_size = 1)
        self.sub_bearing = rospy.Subscriber('/%s/bearing/local' % tello_ns, position_msg, self.cb_bearing_local, queue_size = 1)
        self.sub_bearing = rospy.Subscriber('/%s/bearing/global' % tello_ns, position_msg, self.cb_bearing_global, queue_size = 1)

    def cb_odom(self, odom):
        orien = odom.pose.pose.orientation
        
        rotm = quaternion_matrix([orien.x, orien.y, orien.z, orien.w])  # x, y, z, w;   quaternion is w + xi + yj + zk
        rotm = np.array(rotm)
        rotm = rotm[0:3, 0:3]
        Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # rotate along x 180
        Rz_p = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # rotate along z 90
        Rz_n = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) # rotate along z -90
        rotm = Rz_p.dot(Rx.dot(rotm.dot(Rx.dot(Rz_n)))) # Rz_p -> Rx -> rotm -> Rx -> Rz_n

        self.iRb = rotm

    def cb_bearing_local(self, data):
        direction = np.array(data.position)
        distance = np.sqrt(direction@direction)
        self.bl = direction
        self.nbl = direction / distance

    def cb_bearing_global(self, data):
        direction = np.array(data.position)
        distance = np.sqrt(direction@direction)
        self.bg = direction
        self.nbg = direction / distance

def shutdown_hook():
    print("************in shutdown hook*************")
    pub1.util_hover()
    pub2.util_hover()
    pub3.util_hover()
    rospy.sleep(0.5)
    pub1.util_land()
    pub2.util_land()
    pub3.util_land()

if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)

    # task setting
    tello1_ns = rospy.get_param('~tello1_ns', "tello_601")
    tello2_ns = rospy.get_param('~tello2_ns', "tello_601")
    tello3_ns = rospy.get_param('~tello3_ns', "tello_601")
    
    sub1 = Subs(tello1_ns)
    sub2 = Subs(tello2_ns)
    sub3 = Subs(tello3_ns)
    pub1 = Pubs(tello1_ns)    
    pub2 = Pubs(tello2_ns)
    pub3 = Pubs(tello3_ns)
    pub_sm = PubSm()

    rospy.on_shutdown(shutdown_hook)
    
    Control()
    rospy.spin()