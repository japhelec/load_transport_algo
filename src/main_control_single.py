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

class sFlyupOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_open_finish', 'flyup_open_error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FLYUP_OPEN')

        pubs.util_cmd(0, 0, 0.8, 0)

        rate = rospy.Rate(50) # check image comes in?
        while not rospy.is_shutdown():
            if subs.Ql is not None:
                pubs.util_smach('FLYUP_OPEN', 'WP_ASSIGN')
                return 'flyup_open_finish'
            rate.sleep()
        return 'flyup_open_error'

class sWpAssign(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_assigned', 'wp_assign_finish'], output_keys=['wp_assign_output'])
        Pl = Payload.Pl(ap_id)

        if ap_id == 6:
            wp1 = Pl + np.array([0, -0.2, 0.7])
            wp2 = Pl + np.array([0, -0.1, 0.9])
            wp3 = Pl + np.array([0, 0, 1.1])
        elif ap_id == 0:
            wp1 = Pl + np.array([0, 0.2, 0.7])
            wp2 = Pl + np.array([0, 0.1, 0.9])
            wp3 = Pl + np.array([0, 0, 1.1])
        
        self.wps = [wp1, wp2, wp3]
        # self.wps = [wp1]
        self.wp_count = len(self.wps)
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state WP_ASSIGN')
        if self.counter < self.wp_count:
            userdata.wp_assign_output = self.wps[self.counter]
            self.counter += 1
            pubs.util_smach('WP_ASSIGN %d' % self.counter, 'WP_TRACK %d' % self.counter)
            return 'wp_assigned'
        else:
            pubs.util_smach('WP_ASSIGN', 'LAND')
            return 'wp_assign_finish'

class sWpTracking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_tracking_success', 'wp_tracking_error'], input_keys=['wp_tracking_input'])

        # load fly up control pid gain
        path = os.path.dirname(__file__)
        filepath = str(path) + '/flyup_pid_gain/%s.yml' % tello_ns
        with open(filepath, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        self.Kp_x = data['Kp_x']
        self.Kp_y = data['Kp_y']
        self.Kp_z = data['Kp_z']
        self.Kd_x = data['Kd_x']
        self.Kd_y = data['Kd_y']
        self.Kd_z = data['Kd_z']
        self.Ki_x = data['Ki_x']
        self.Ki_y = data['Ki_y']
        self.Ki_z = data['Ki_z']

    def execute(self, userdata):
        rospy.loginfo('Executing state WP_TRACKING')  

        desired_Ql = userdata.wp_tracking_input
        preErr_x = 0.0
        preErr_y = 0.0
        preErr_z = 0.0
        sumErr_x = 0.0
        sumErr_y = 0.0
        sumErr_z = 0.0
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            err = desired_Ql - subs.Ql

            if (err.dot(err) < 0.0025): # distance < 5 cm
                # pubs.util_smach('WP_TRACK', 'WP_ASSIGN')
                return 'wp_tracking_success'

            ###### PID #######
            sumErr_x = sumErr_x + err[0]
            sumErr_y = sumErr_y + err[1]
            sumErr_z = sumErr_z + err[2]
            dErr_x = err[0] - preErr_x
            dErr_y = err[1] - preErr_y
            dErr_z = err[2] - preErr_z
            ux = self.Kp_x * err[0] + self.Ki_x * sumErr_x + self.Kd_x * dErr_x
            uy = self.Kp_y * err[1] + self.Ki_y * sumErr_y + self.Kd_y * dErr_y
            uz = self.Kp_z * err[2] + self.Ki_z * sumErr_z + self.Kd_z * dErr_z
            u = np.array([ux, uy, uz])
            preErr_x = err[0]
            preErr_y = err[1]
            preErr_z = err[2]
            ###### PID #######
            
            u = subs.bRc.dot(subs.cRm.dot(subs.mRl.dot(u)))
            pubs.util_cmd(u[0], u[1], u[2], 0)
            rate.sleep()
        
        return 'wp_tracking_error'

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
        return 'fake_finish'

class Control():
    def __init__(self):        
        self.sm_top = smach.StateMachine(outcomes=['control_finish'])
        with self.sm_top:
            smach.StateMachine.add('WARMUP', sWarmup(), 
                transitions={'warmup_finish':'FLYUP_OPEN'})
            smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                transitions={'flyup_open_finish':'FLYUP_CONTROL', 'flyup_open_error':'control_finish'})

            self.sm_flyup_control = smach.StateMachine(outcomes=['flyup_control_finish', 'flyup_control_error'])
            self.sm_flyup_control.userdata.desired_Ql = None
            with self.sm_flyup_control:
                smach.StateMachine.add('WP_ASSIGN', sWpAssign(), 
                    transitions={'wp_assigned':'WP_TRACK', 'wp_assign_finish':'flyup_control_finish'},
                    remapping={'wp_assign_output':'desired_Ql'})
                smach.StateMachine.add('WP_TRACK', sWpTracking(), 
                    transitions={'wp_tracking_success':'WP_ASSIGN', 'wp_tracking_error': 'flyup_control_error'},
                    remapping={'wp_tracking_input':'desired_Ql'})

            smach.StateMachine.add('FLYUP_CONTROL', self.sm_flyup_control, 
                transitions={'flyup_control_finish':'LAND', 'flyup_control_error':'control_finish'})
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
        self.bRc = Drone.bRc(tello_ns)
        self.marker_id = None
        self.Ql = None
        self.cRm = None
        self.mRl = None
        
        self.sub_odom = rospy.Subscriber('/%s/odom' % tello_ns, Odometry, self.cb_odom, queue_size = 1)
        self.sub_cRm = rospy.Subscriber('/%s/cRm' % tello_ns, cRm_msg, self.cb_cRm, queue_size = 1)
        # self.sub_Mc = rospy.Subscriber('/%s/Mc' % tello_ns, Mc_msg, self.cb_Mc, queue_size = 1)
        self.sub_Ql = rospy.Subscriber('/%s/Ql' % tello_ns, position_msg, self.cb_Ql, queue_size = 1)

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
        self.mRl = Payload.mRl(data.marker_id)

    def cb_Ql(self, data):
        self.Ql = np.array(data.position)
  
if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)

    # task setting
    tello_ns = rospy.get_param('~tello_ns', "tello_601")
    ap_id = int(rospy.get_param('~ap_id', "tello_601"))
    subs = Subs()
    pubs = Pubs()    
    
    Control()
    rospy.spin()
