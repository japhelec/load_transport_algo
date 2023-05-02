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
        sub1.bg = None
        sub1.distance = None
        sub2.bl = None
        sub2.bg = None
        sub2.distance = None
        sub3.bl = None
        sub3.bg = None
        sub3.distance = None
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            # if ((sub1.bearing is not None) and (sub2.bearing is not None) and (sub3.bearing is not None)):
            #     if ((self.pid1r.check(self.bearing2theta(sub1.bearing))) and (self.pid2r.check(self.bearing2theta(sub1.bearing))) and (self.pid3r.check(self.bearing2theta(sub1.bearing)))):
            #         break
            
            # u1z = self.pid1z.update(sub1.h)
            # u2z = self.pid2z.update(sub2.h)
            # u3z = self.pid3z.update(sub3.h)

            # pub1.util_h_err(self.pid1z.err)
            # pub2.util_h_err(self.pid2z.err)
            # pub3.util_h_err(self.pid3z.err)

            if sub1.bl is None:
                u1z = 0
                u1r = 0.4
            else:
                bl = np.copy(sub1.bl)
                bl[1] = 0
                psi = bl@np.array([0,0,1])
                psi = psi / np.sqrt(bl@bl)
                psi = np.arccos(psi)
                psi = -np.sign(bl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi
                u1r = self.pid_yaw_1.update(psi)         # desired: 0

                be = (Drone.camTilt)@sub1.bl
                be[0] = 0
                phi = be@np.array([0,0,1])
                phi = phi / np.sqrt(be@be)
                phi = np.arccos(phi)
                phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
                u1z = self.pid_pitch_1.update(phi)         # desired: 0

            if sub2.bl is None:
                u2z = 0
                u2r = 0.4
            else:
                bl = np.copy(sub2.bl)
                bl[1] = 0
                psi = bl@np.array([0,0,1])
                psi = psi / np.sqrt(bl@bl)
                psi = np.arccos(psi)
                psi = -np.sign(bl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi
                u2r = self.pid_yaw_2.update(psi)         # desired: 0

                be = (Drone.camTilt)@sub2.bl
                be[0] = 0
                phi = be@np.array([0,0,1])
                phi = phi / np.sqrt(be@be)
                phi = np.arccos(phi)
                phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
                u2z = self.pid_pitch_2.update(phi)         # desired: 0

            if sub3.bl is None:
                u3z = 0
                u3r = 0.4
            else:
                bl = np.copy(sub3.bl)
                bl[1] = 0
                psi = bl@np.array([0,0,1])
                psi = psi / np.sqrt(bl@bl)
                psi = np.arccos(psi)
                psi = -np.sign(bl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi
                u3r = self.pid_yaw_3.update(psi)         # desired: 0

                be = (Drone.camTilt)@sub3.bl
                be[0] = 0
                phi = be@np.array([0,0,1])
                phi = phi / np.sqrt(be@be)
                phi = np.arccos(phi)
                phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
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

    def bearing2theta(self, bearing):
        bearing[1] = 0
        theta = bearing@np.array([0,0,1])
        theta = theta / np.sqrt(bearing@bearing)
        theta = np.arccos(theta)*180/np.pi
        theta = -np.sign(bearing[0]) * theta

        return theta

class sFormationControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fc_finish'])

        # load pid gain
        kpr = float(rospy.get_param('~yaw_kp', "0.025"))
        kir = float(rospy.get_param('~yaw_ki', "0.0"))
        kdr = float(rospy.get_param('~yaw_kd', "0.0"))
        self.pid1r = PID_single_var(kpr, kir, kdr)
        self.pid2r = PID_single_var(kpr, kir, kdr)
        self.pid3r = PID_single_var(kpr, kir, kdr)

        # set target
        yaw_tol = float(rospy.get_param('~yaw_tol', "5"))
        self.pid1r.setTarget(0)
        self.pid2r.setTarget(0)
        self.pid3r.setTarget(0)

        d1 = -90*np.pi/180
        self.bg1_d = np.array([[np.cos(d1),-np.sin(d1),0],[np.sin(d1),np.cos(d1),0],[0,0,1]])@(Drone.bRc)@(Drone.camTilt)@np.array([0, -0.277, 1])
        d2 = 30*np.pi/180
        self.bg2_d = np.array([[np.cos(d2),-np.sin(d2),0],[np.sin(d2),np.cos(d2),0],[0,0,1]])@(Drone.bRc)@(Drone.camTilt)@np.array([0, -0.277, 1])
        d3 = 150*np.pi/180
        self.bg3_d = np.array([[np.cos(d3),-np.sin(d3),0],[np.sin(d3),np.cos(d3),0],[0,0,1]])@(Drone.bRc)@(Drone.camTilt)@np.array([0, -0.277, 1])

        # print("=================")
        # print(self.bg1_d)
        # print(self.bg2_d)
        # print(self.bg3_d)
        # print("=================")

    def execute(self, userdata):
        rospy.loginfo('Executing state FORMATION CONTROL')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():           
            # yaw control
            bl1 = sub1.bearing
            theta = self.bearing2theta(bl1)
            u1r = self.pid1r.update(theta)
            pub1.util_yaw_error(self.pid1r.err)

            bl2 = sub2.bearing
            theta = self.bearing2theta(bl2)
            u2r = self.pid2r.update(theta)
            pub2.util_yaw_error(self.pid2r.err)

            bl3 = sub3.bearing
            theta = self.bearing2theta(bl3)
            u3r = self.pid3r.update(theta)
            pub3.util_yaw_error(self.pid3r.err)

            # formation control
            bg1 = (sub1.iRb)@(Drone.bRc)@(Drone.camTilt)@bl1
            bg2 = (sub2.iRb)@(Drone.bRc)@(Drone.camTilt)@bl2
            bg3 = (sub3.iRb)@(Drone.bRc)@(Drone.camTilt)@bl3
            pub1.util_bearing_global(bg1)
            pub2.util_bearing_global(bg2)
            pub3.util_bearing_global(bg3)
            
            err1 = self.bearingDiff(bg1, self.bg1_d)
            err2 = self.bearingDiff(bg2, self.bg2_d)
            err3 = self.bearingDiff(bg3, self.bg3_d)
            pub1.util_bearing_error(err1)
            pub2.util_bearing_error(err2)
            pub3.util_bearing_error(err3)

            # if ((err1 < 0.08) and (err2 < 0.08) and (err3 < 0.08)):
            #     break

            K = 0.3
            # u1 = self.bg1_d - bg1
            # u1[0] = u1[0]*K
            # u1[1] = u1[1]*K
            # u2 = self.bg2_d - bg2
            # u2[0] = u2[0]*K
            # u2[1] = u2[1]*K
            # u3 = self.bg3_d - bg3
            # u3[0] = u3[0]*K
            # u3[1] = u3[1]*K
            u1 = K*(self.bg1_d - bg1)
            u2 = K*(self.bg2_d - bg2)
            u3 = K*(self.bg3_d - bg3)
            u1 = (sub1.iRb.T)@u1
            u2 = (sub2.iRb.T)@u2
            u3 = (sub3.iRb.T)@u3

            pub1.util_cmd(u1[0], u1[1], u1[2], u1r)
            pub2.util_cmd(u2[0], u2[1], u2[2], u2r)
            pub3.util_cmd(u3[0], u3[1], u3[2], u3r)
            rate.sleep()
        
        pub_sm.util_smach('FORMATION_CONTROL', 'LAND')
        return 'fc_finish'

    def bearing2theta(self, bearing):
        bearing[1] = 0
        theta = bearing@np.array([0,0,1])
        theta = theta / np.sqrt(bearing@bearing)
        theta = np.arccos(theta)*180/np.pi
        theta = -np.sign(bearing[0]) * theta

        return theta

    def bearingDiff(self, bgd, bg):
        theta = bgd@bg/(np.linalg.norm(bgd)*np.linalg.norm(bg))
        return theta

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
            # smach.StateMachine.add('WARMUP', sWarmup(), 
            #     transitions={'warmup_finish':'TAKEOFF'})
            # smach.StateMachine.add('TAKEOFF', sTakeoff(), 
            #     transitions={'takeoff_finish':'YAW_SEARCH'})
            # smach.StateMachine.add('WARMUP', sWarmup(), 
            #     transitions={'warmup_finish':'LAND'})

            smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                transitions={'flyup_open_finish':'YAW_SEARCH'})
            smach.StateMachine.add('YAW_SEARCH', sYawSearch(), 
                transitions={'ys_finish':'LAND'})
            # smach.StateMachine.add('YAW_SEARCH', sYawSearch(), 
            #     transitions={'ys_finish':'FORMATION_CONTROL'})
            # smach.StateMachine.add('FORMATION_CONTROL', sFormationControl(), 
            #     transitions={'fc_finish':'LAND'})

            # smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
            #     transitions={'flyup_open_finish':'HOVER'})
            # smach.StateMachine.add('HOVER', sHover(), 
            #     transitions={'hover_finish':'LEFT'})
            # smach.StateMachine.add('LEFT', sLeft(), 
            #     transitions={'left_finish':'RIGHT'})
            # smach.StateMachine.add('RIGHT', sRight(), 
            #     transitions={'right_finish':'LAND'})

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
        self.pub_pitch_error = rospy.Publisher('/%s/yaw/error' % tello_ns, position_msg, queue_size=1)
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
        self.bg = None
        self.distance = None
        
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
        self.distance = np.sqrt(direction@direction)
        self.bl = direction / self.distance

    def cb_bearing_global(self, data):
        direction = np.array(data.position)
        distance = np.sqrt(direction@direction)
        self.bg = direction / distance

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