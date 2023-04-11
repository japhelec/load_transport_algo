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
from load_transport.msg import cRm_msg, Mc_msg, position_msg, state_machine_msg
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
        pub_sm.util_smach('WARM_UP', 'FLYUP_UNTIL')
        return 'warmup_finish'

class sFlyupUntil(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_until_finish', 'flyup_until_error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FLYUP_UNTIL')

        pub1.util_cmd(0, 0, 0.8, 0)
        pub2.util_cmd(0, 0, 0.8, 0)
        pub3.util_cmd(0, 0, 0.8, 0)

        rate = rospy.Rate(50) # check image comes in?
        while not rospy.is_shutdown():
            if (sub1.Ql is not None) and (sub2.Ql is not None) and (sub3.Ql is not None):
                pub_sm.util_smach('FLYUP_UNTIL', 'WP_ASSIGN')
                return 'flyup_until_finish'
            rate.sleep()
        return 'flyup_until_error'

class sWpAssign(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_assigned', 'wp_assign_finish'], output_keys=['wp_assign_output1', 'wp_assign_output2', 'wp_assign_output3'])
        Pl1 = Payload.Pl(ap1_id)
        Pl2 = Payload.Pl(ap2_id)
        Pl3 = Payload.Pl(ap3_id)

        # wp1 = Pl1 + np.array([0, 0.2, 0.7])
        # wp2 = Pl1 + np.array([0, 0.1, 0.9])
        wp3 = Pl1 + np.array([0, 0, 0.7])
        self.wps1 = [wp3]
        # self.wps1 = [wp1, wp2, wp3]
        # wp1 = Pl2 + np.array([0, -0.2, 0.7])
        # wp2 = Pl2 + np.array([0, -0.1, 0.9])
        wp3 = Pl2 + np.array([0, 0, 0.7])
        self.wps2 = [wp3]
        # self.wps2 = [wp1, wp2, wp3]
        wp3 = Pl3 + np.array([0, 0, 0.7])
        self.wps3 = [wp3]
        
        self.wp_count = len(self.wps1)
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state WP_ASSIGN')
        if self.counter < self.wp_count:
            userdata.wp_assign_output1 = self.wps1[self.counter]
            userdata.wp_assign_output2 = self.wps2[self.counter]
            userdata.wp_assign_output3 = self.wps3[self.counter]
            self.counter += 1
            pub_sm.util_smach('WP_ASSIGN %d' % self.counter, 'WP_TRACK %d' % self.counter)
            return 'wp_assigned'
        else:
            pub_sm.util_smach('WP_ASSIGN', 'LAND')
            return 'wp_assign_finish'

class sWpTracking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_tracking_success', 'wp_tracking_error'], input_keys=['wp_tracking_input1', 'wp_tracking_input2', 'wp_tracking_input3'])

        # load fly up control pid gain
        kpx = float(rospy.get_param('~flyup_kpx', "1.2"))
        kpy = float(rospy.get_param('~flyup_kpy', "1.2"))
        kpz = float(rospy.get_param('~flyup_kpz', "1.2"))
        kix = float(rospy.get_param('~flyup_kix', "0.03"))
        kiy = float(rospy.get_param('~flyup_kiy', "0.03"))
        kiz = float(rospy.get_param('~flyup_kiz', "0.03"))
        kdx = float(rospy.get_param('~flyup_kdx', "0.001"))
        kdy = float(rospy.get_param('~flyup_kdy', "0.001"))
        kdz = float(rospy.get_param('~flyup_kdz', "0.001"))

        self.pid1 = PID(
            kpx, kpy, kpz,
            kix, kiy, kiz, 
            kdx, kdy, kdz)

        self.pid2 = PID(
            kpx, kpy, kpz,
            kix, kiy, kiz, 
            kdx, kdy, kdz)

        self.pid3 = PID(
            kpx, kpy, kpz,
            kix, kiy, kiz, 
            kdx, kdy, kdz)

    def execute(self, userdata):
        rospy.loginfo('Executing state WP_TRACKING')          
        flyup_tol = float(rospy.get_param('~flyup_tol', "0.05"))
        self.pid1.setTarget(userdata.wp_tracking_input1, flyup_tol)
        self.pid2.setTarget(userdata.wp_tracking_input2, flyup_tol)
        self.pid3.setTarget(userdata.wp_tracking_input3, flyup_tol)

        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            if self.pid1.check(sub1.Ql) and self.pid2.check(sub2.Ql) and self.pid3.check(sub3.Ql): 
                # pubs.util_smach('WP_TRACK', 'WP_ASSIGN')
                return 'wp_tracking_success'
            
            u1 = self.pid1.update(sub1.Ql)
            u2 = self.pid2.update(sub2.Ql)
            u3 = self.pid3.update(sub3.Ql)
            u1 = sub1.bRc.dot(sub1.cRm.dot(Payload.mRl().dot(u1)))
            u2 = sub2.bRc.dot(sub2.cRm.dot(Payload.mRl().dot(u2)))
            u3 = sub3.bRc.dot(sub3.cRm.dot(Payload.mRl().dot(u3)))

            pub1.util_Ql_err(self.pid1.err)
            pub2.util_Ql_err(self.pid2.err)
            pub3.util_Ql_err(self.pid3.err)
            pub1.util_cmd(u1[0], u1[1], u1[2], 0)
            pub2.util_cmd(u2[0], u2[1], u2[2], 0)
            pub3.util_cmd(u3[0], u3[1], u3[2], 0)
            rate.sleep()
        
        return 'wp_tracking_error'

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

class sHeightControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hc_finish'])

        # load fly up control pid gain
        kpz = float(rospy.get_param('~height_kpz', "2.0"))
        kiz = float(rospy.get_param('~height_kiz', "2.0"))
        kdz = float(rospy.get_param('~height_kdz', "2.0"))

        self.pid1 = PID_z(kpz, kiz, kdz)
        self.pid2 = PID_z(kpz, kiz, kdz)
        self.pid3 = PID_z(kpz, kiz, kdz)

        # set target height
        height_d = float(rospy.get_param('~height_d', "1.2"))
        height_tol = float(rospy.get_param('~height_tol', "0.05"))
        self.pid1.setTarget(height_d, height_tol)
        self.pid2.setTarget(height_d, height_tol)
        self.pid3.setTarget(height_d, height_tol)

    def execute(self, userdata):
        rospy.loginfo('Executing state HEIGHT CONTROL')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():

            if (self.pid1.check(sub1.h) and (self.pid2.check(sub2.h)) and (self.pid3.check(sub3.h))):
                break
            
            u1 = self.pid1.update(sub1.h)
            u2 = self.pid2.update(sub2.h)
            u3 = self.pid3.update(sub3.h)

            pub1.util_h_err(self.pid1.err)
            pub2.util_h_err(self.pid2.err)
            pub3.util_h_err(self.pid3.err)

            pub1.util_cmd(0, 0, u1, 0)
            pub2.util_cmd(0, 0, u2, 0)
            pub3.util_cmd(0, 0, u3, 0)
            rate.sleep()
        
        pub_sm.util_smach('HEIGHT_CONTROL', 'YAW_SEARCH')
        return 'hc_finish'

class sYawSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ys_finish'])

        # load pid gain
        kpz = float(rospy.get_param('~height_kpz', "2.0"))
        kiz = float(rospy.get_param('~height_kiz', "2.0"))
        kdz = float(rospy.get_param('~height_kdz', "2.0"))
        self.pid1z = PID_single_var(kpz, kiz, kdz)
        self.pid2z = PID_single_var(kpz, kiz, kdz)
        self.pid3z = PID_single_var(kpz, kiz, kdz)

        kpr = float(rospy.get_param('~yaw_kp', "0.025"))
        kir = float(rospy.get_param('~yaw_ki', "0.0"))
        kdr = float(rospy.get_param('~yaw_kd', "0.0"))
        self.pid1r = PID_single_var(kpr, kir, kdr)
        self.pid2r = PID_single_var(kpr, kir, kdr)
        self.pid3r = PID_single_var(kpr, kir, kdr)

        # set target
        height_d = float(rospy.get_param('~height_d', "1.2"))
        self.pid1z.setTarget(height_d)
        self.pid2z.setTarget(height_d)
        self.pid3z.setTarget(height_d)
        yaw_tol = float(rospy.get_param('~yaw_tol', "5"))
        self.pid1r.setTarget(0, yaw_tol)
        self.pid2r.setTarget(0, yaw_tol)
        self.pid3r.setTarget(0, yaw_tol)

    def execute(self, userdata):
        rospy.loginfo('Executing state YAW SEARCH')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            if ((sub1.bearing is not None) and (sub2.bearing is not None) and (sub3.bearing is not None)):
                if ((self.pid1r.check(sub1.bearing)) and (self.pid2r.check(sub2.bearing)) and (self.pid3r.check(sub3.bearing))):
                    break
            
            u1z = self.pid1z.update(sub1.h)
            u2z = self.pid2z.update(sub2.h)
            u3z = self.pid3z.update(sub3.h)

            pub1.util_h_err(self.pid1z.err)
            pub2.util_h_err(self.pid2z.err)
            pub3.util_h_err(self.pid3z.err)

            if sub1.bearing is None:
                u1r = 0.4
            else:
                bearing = sub1.bearing
                bearing[1] = 0
                theta = bearing@np.array([0,0,1])
                theta = theta / np.sqrt(bearing@bearing)
                theta = np.arccos(theta)*180/np.pi
                theta = -np.sign(bearing[0]) * theta
                u1r = self.pid1r.update(theta)
                pub1.pub_yaw_error(self.pid1r.err)

            if sub2.bearing is None:
                u2r = 0.4
            else:
                bearing = sub2.bearing
                bearing[1] = 0
                theta = bearing@np.array([0,0,1])
                theta = theta / np.sqrt(bearing@bearing)
                theta = np.arccos(theta)*180/np.pi
                theta = -np.sign(bearing[0]) * theta
                u2r = self.pid2r.update(theta)
                pub2.pub_yaw_error(self.pid2r.err)

            if sub3.bearing is None:
                u3r = 0.4
            else:
                bearing = sub3.bearing
                bearing[1] = 0
                theta = bearing@np.array([0,0,1])
                theta = theta / np.sqrt(bearing@bearing)
                theta = np.arccos(theta)*180/np.pi
                theta = -np.sign(bearing[0]) * theta
                u3r = self.pid3r.update(theta)
                pub3.pub_yaw_error(self.pid3r.err)

            pub1.util_cmd(0, 0, u1z, u1r)
            pub2.util_cmd(0, 0, u2z, u2r)
            pub3.util_cmd(0, 0, u3z, u3r)
            rate.sleep()
        
        pub_sm.util_smach('YAW_SEARCH', 'LAND')
        return 'ys_finish'

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
            #     transitions={'warmup_finish':'LAND'})

            smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                transitions={'flyup_open_finish':'HEIGHT_CONTROL'})
            smach.StateMachine.add('HEIGHT_CONTROL', sHeightControl(), 
                transitions={'hc_finish':'YAW_SEARCH'})
            smach.StateMachine.add('YAW_SEARCH', sYawSearch(), 
                transitions={'ys_finish':'LAND'})

            # smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
            #     transitions={'flyup_open_finish':'HOVER'})
            # smach.StateMachine.add('HOVER', sHover(), 
            #     transitions={'hover_finish':'LAND'})

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
        self.pub_cmd_vel = rospy.Publisher('/%s/cmd_vel' % tello_ns, Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/%s/land' % tello_ns, Empty, queue_size=1)
        self.pub_Ql_error = rospy.Publisher('/%s/Ql/error' % tello_ns, position_msg, queue_size=1)
        self.pub_h_error = rospy.Publisher('/%s/height/error' % tello_ns, position_msg, queue_size=1)
        self.pub_yaw_error = rospy.Publisher('/%s/yaw/error' % tello_ns, position_msg, queue_size=1)

    def util_cmd(self, x, y, z, yaw):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = yaw
        self.pub_cmd_vel.publish(msg)

    def util_motor_on(self):
        self.pub_motor_on.publish()

    def util_hover(self):
        self.util_cmd(0,0,0,0)

    def util_land(self):
        self.pub_land.publish()

    def util_Ql_err(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = err
        self.pub_Ql_error.publish(msg)

    def util_h_err(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_h_error.publish(msg)

    def pub_yaw_error(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_yaw_error.publish(msg)

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
        self.Q_i = np.array([0,0,0])
        self.iRb = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.bRc = Drone.bRc(tello_ns)
        self.marker_id = None
        self.Ql = None
        self.cRm = None
        self.mRl = None
        self.h = 0
        self.bearing = None
        
        self.sub_odom = rospy.Subscriber('/%s/odom' % tello_ns, Odometry, self.cb_odom, queue_size = 1)
        self.sub_cRm = rospy.Subscriber('/%s/cRm/filtered' % tello_ns, cRm_msg, self.cb_cRm, queue_size = 1)
        self.sub_Ql = rospy.Subscriber('/%s/Ql/filtered' % tello_ns, position_msg, self.cb_Ql, queue_size = 1)
        self.sub_height = rospy.Subscriber('/%s/height/filtered' % tello_ns, position_msg, self.cb_height, queue_size = 1)
        self.sub_bearing = rospy.Subscriber('/%s/bearing/local' % tello_ns, position_msg, self.cb_bearing, queue_size = 1)

    def cb_odom(self, odom):
        pos = odom.pose.pose.position
        orien = odom.pose.pose.orientation
        
        rotm = quaternion_matrix([orien.x, orien.y, orien.z, orien.w])  # x, y, z, w;   quaternion is w + xi + yj + zk
        rotm = np.array(rotm)
        rotm = rotm[0:3, 0:3]
        Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # rotate along x 180
        Rz_p = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # rotate along z 90
        Rz_n = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) # rotate along z -90
        rotm = Rz_p.dot(Rx.dot(rotm.dot(Rx.dot(Rz_n)))) # Rz_p -> Rx -> rotm -> Rx -> Rz_n

        self.Q_i = np.array([pos.y, pos.x, -pos.z])
        self.iRb = rotm

    def cb_cRm(self, data):
        self.marker_id = data.marker_id
        
        rvec = np.array([[data.rvec]])
        cRm, jacob = cv2.Rodrigues(rvec) 
        self.cRm = cRm
        self.mRl = Payload.mRl()

    def cb_Ql(self, data):
        self.Ql = np.array(data.position)

    def cb_height(self, data):
        self.h = data.position[2]

    def cb_bearing(self, data):
        self.bearing = np.array(data.position)

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
    ap1_id = Drone.ns2id(tello1_ns)
    tello2_ns = rospy.get_param('~tello2_ns', "tello_601")
    ap2_id = Drone.ns2id(tello2_ns)
    tello3_ns = rospy.get_param('~tello3_ns', "tello_601")
    ap3_id = Drone.ns2id(tello3_ns)
    

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