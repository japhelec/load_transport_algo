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
from util_PID_control import PID, PID_z, PID_single_var


class sWarmup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['warmup_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WARMUP')

        rospy.sleep(5.0)
        pubs.util_motor_on()
        rospy.sleep(5.0)
        pubs.util_smach('WARM_UP', 'FLYUP_OPEN')
        return 'warmup_finish'

class sLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['land_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LAND')

        pubs.util_hover()
        rospy.sleep(0.5)
        pubs.util_land()
        return 'land_finish'

class sFake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fake_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FAKE')

        pubs.util_cmd(0, 0, 0, 0.2)
        rospy.sleep(20)

        return 'fake_finish'

class sFlyupOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_open_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FLYUP_OPEN')

        thrust = float(rospy.get_param('~lift_thrust', "1.5"))
        duration = float(rospy.get_param('~lift_duration', "3"))

        pubs.util_cmd(0, 0, thrust, 0)
        rospy.sleep(duration)

        return 'flyup_open_finish'

# class sInfiniteHover(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['hover_finish'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state INFINITE_HOVER')

#         rate = rospy.Rate(15) 
#         while not rospy.is_shutdown():
            
#             pubs.util_hover()
#             rate.sleep()
        
#         return 'hover_finish'

class sHover(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hover_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HOVER')
        pubs.util_hover()
        rospy.sleep(20.0)
        return 'hover_finish'

class sStabilize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stab_finish'])

        # clear
        subs.bl = None
        subs.bg = None
        subs.distance = None

        # load pid gain
        kp_yaw = float(rospy.get_param('~yaw_kp', "0.025"))
        ki_yaw = float(rospy.get_param('~yaw_ki', "0.025"))
        kd_yaw = float(rospy.get_param('~yaw_kd', "0.025"))
        self.pid_yaw = PID_single_var(kp_yaw, ki_yaw, kd_yaw)

        kp_pitch = float(rospy.get_param('~pitch_kp', "0.025"))
        ki_pitch = float(rospy.get_param('~pitch_ki', "0.025"))
        kd_pitch = float(rospy.get_param('~pitch_kd', "0.025"))
        self.pid_pitch = PID_single_var(kp_pitch, ki_pitch, kd_pitch)

        # set target 
        self.pid_yaw.setTarget(0)
        self.pid_pitch.setTarget(-3*np.pi/180)

    def execute(self, userdata):
        rospy.loginfo('Executing state STABILIZE')          
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():

            if subs.bl is None:
                u3 = 0
                u4 = 0.4
            else:
                bl = np.copy(subs.bl)
                bl[1] = 0
                psi = bl@np.array([0,0,1])
                psi = psi / np.sqrt(bl@bl)
                psi = np.arccos(psi)
                psi = -np.sign(bl[0]) * psi   # variable: difference between z axis and taregt ==> 0 - psi
                u4 = self.pid_yaw.update(psi)         # desired: 0

                be = (Drone.camTilt)@subs.bl
                be[0] = 0
                phi = be@np.array([0,0,1])
                phi = phi / np.sqrt(be@be)
                phi = np.arccos(phi)
                phi = np.sign(be[1]) * phi   # variable: difference between z axis and taregt ==> 0 - phi
                u3 = self.pid_pitch.update(phi)         # desired: 0

            pubs.util_yaw_err(self.pid_yaw.err)
            pubs.util_pitch_err(self.pid_pitch.err)
            pubs.util_cmd(0, 0, u3, u4)
            rate.sleep()
        
        return 'stab_finish'

# class Control():
#     def __init__(self):        
#         self.sm_top = smach.StateMachine(outcomes=['control_finish'])
#         with self.sm_top:
#             smach.StateMachine.add('WARMUP', sWarmup(), 
#                 transitions={'warmup_finish':'FLYUP_UNTIL'})
#             smach.StateMachine.add('FLYUP_UNTIL', sFlyupUntil(), 
#                 transitions={'flyup_until_finish':'FLYUP_CONTROL', 'flyup_until_error':'control_finish'})

#             self.sm_flyup_control = smach.StateMachine(outcomes=['flyup_control_finish', 'flyup_control_error'])
#             self.sm_flyup_control.userdata.desired_Ql = None
#             with self.sm_flyup_control:
#                 smach.StateMachine.add('WP_ASSIGN', sWpAssign(), 
#                     transitions={'wp_assigned':'WP_TRACK', 'wp_assign_finish':'flyup_control_finish'},
#                     remapping={'wp_assign_output':'desired_Ql'})
#                 smach.StateMachine.add('WP_TRACK', sWpTracking(), 
#                     transitions={'wp_tracking_success':'WP_ASSIGN', 'wp_tracking_error': 'flyup_control_error'},
#                     remapping={'wp_tracking_input':'desired_Ql'})

#             smach.StateMachine.add('FLYUP_CONTROL', self.sm_flyup_control, 
#                 transitions={'flyup_control_finish':'LAND', 'flyup_control_error':'control_finish'})
#             smach.StateMachine.add('LAND', sLand(), 
#                 transitions={'land_finish':'control_finish'})
#         smach_thread = threading.Thread(target=self.sm_top.execute, daemon = True)
#         smach_thread.start()

class Control():
    def __init__(self):        
        self.sm_top = smach.StateMachine(outcomes=['control_finish'])
        with self.sm_top:
            smach.StateMachine.add('WARMUP', sWarmup(), 
                transitions={'warmup_finish':'FLYUP_OPEN'})
            # smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                # transitions={'flyup_open_finish':'HOVER'})
            # smach.StateMachine.add('HOVER', sHover(), 
            #     transitions={'hover_finish':'LAND'})
            # smach.StateMachine.add('HOVER', sInfiniteHover(), 
                # transitions={'hover_finish':'LAND'})
            # smach.StateMachine.add('HOVER', sFake(), 
                # transitions={'fake_finish':'LAND'})

            
            smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                transitions={'flyup_open_finish':'STABILIZE'})
            # # smach.StateMachine.add('STABILIZE', sStabilizeZ(), 
            # #     transitions={'stab_finish':'LAND'})
            smach.StateMachine.add('STABILIZE', sStabilize(), 
                transitions={'stab_finish':'LAND'})
            smach.StateMachine.add('LAND', sLand(), 
                transitions={'land_finish':'control_finish'})
        smach_thread = threading.Thread(target=self.sm_top.execute, daemon = True)
        smach_thread.start()

class Pubs():
    def __init__(self):
        # publisher
        self.pub_motor_on = rospy.Publisher('/%s/manual_takeoff' % tello_ns, Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/%s/cmd_vel' % tello_ns, Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/%s/land' % tello_ns, Empty, queue_size=1)
        self.pub_smach = rospy.Publisher('/state_transition', state_machine_msg, queue_size=1)
        self.pub_yaw_error = rospy.Publisher('/%s/yaw/error' % tello_ns, position_msg, queue_size=1)
        self.pub_pitch_error = rospy.Publisher('/%s/pitch/error' % tello_ns, position_msg, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

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

    def util_smach(self, state_before, state_after):
        msg = state_machine_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.before = state_before
        msg.after = state_after
        self.pub_smach.publish(msg)

    def util_yaw_err(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_yaw_error.publish(msg)

    def util_pitch_err(self, err):
        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = np.array([0,0,err])
        self.pub_pitch_error.publish(msg)

    def shutdown_hook(self):
        print("************in shutdown hook*************")
        self.util_hover()
        rospy.sleep(0.5)
        self.util_land()

class Subs():
    # subscriber and perception
    def __init__(self):
        self.Q_i = np.array([0,0,0])
        self.iRb = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.bl = None
        self.bg = None
        self.distance = None

        self.sub_odom = rospy.Subscriber('/%s/odom' % tello_ns, Odometry, self.cb_odom, queue_size = 1)
        self.sub_bearing_local = rospy.Subscriber('/%s/bearing/local' % tello_ns, position_msg, self.cb_bearing_local, queue_size = 1)
        self.sub_bearing_global = rospy.Subscriber('/%s/bearing/global' % tello_ns, position_msg, self.cb_bearing_global, queue_size = 1)


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

    def cb_bearing_local(self, data):
        direction = np.array(data.position)
        self.distance = np.sqrt(direction@direction)
        self.bl = direction / self.distance

    def cb_bearing_global(self, data):
        direction = np.array(data.position)
        distance = np.sqrt(direction@direction)
        self.bg = direction / distance


if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)

    # task setting
    tello_ns = rospy.get_param('~tello_ns', "tello_601")
    ap_id = Drone.ns2id(tello_ns)
    subs = Subs()
    pubs = Pubs()    
    
    Control()
    rospy.spin()
