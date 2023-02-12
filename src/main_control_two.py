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
        pub1.util_motor_on()
        pub2.util_motor_on()
        rospy.sleep(5.0)
        pub_sm.util_smach('WARM_UP', 'FLYUP_OPEN')
        return 'warmup_finish'

class sFlyupOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_open_finish', 'flyup_open_error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FLYUP_OPEN')

        pub1.util_cmd(0, 0, 0.8, 0)
        pub2.util_cmd(0, 0, 0.8, 0)

        rate = rospy.Rate(50) # check image comes in?
        while not rospy.is_shutdown():
            if (sub1.Ql is not None) and (sub2.Ql is not None):
                pub_sm.util_smach('FLYUP_OPEN', 'WP_ASSIGN')
                return 'flyup_open_finish'
            rate.sleep()
        return 'flyup_open_error'

class sWpAssign(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_assigned', 'wp_assign_finish'], output_keys=['wp_assign_output1', 'wp_assign_output2'])
        Pl1 = Payload.Pl(ap1_id)
        Pl2 = Payload.Pl(ap2_id)

        wp1 = Pl1 + np.array([0, 0.2, 0.7])
        wp2 = Pl1 + np.array([0, 0.1, 0.9])
        wp3 = Pl1 + np.array([0, 0, 1.1])
        # self.wps1 = [wp3]
        self.wps1 = [wp1, wp2, wp3]
        wp1 = Pl2 + np.array([0, -0.2, 0.7])
        wp2 = Pl2 + np.array([0, -0.1, 0.9])
        wp3 = Pl2 + np.array([0, 0, 1.1])
        # self.wps2 = [wp3]
        self.wps2 = [wp1, wp2, wp3]
        
        self.wp_count = len(self.wps1)
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state WP_ASSIGN')
        if self.counter < self.wp_count:
            userdata.wp_assign_output1 = self.wps1[self.counter]
            userdata.wp_assign_output2 = self.wps2[self.counter]
            self.counter += 1
            pub_sm.util_smach('WP_ASSIGN %d' % self.counter, 'WP_TRACK %d' % self.counter)
            return 'wp_assigned'
        else:
            pub_sm.util_smach('WP_ASSIGN', 'LAND')
            return 'wp_assign_finish'

class sWpTracking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wp_tracking_success', 'wp_tracking_error'], input_keys=['wp_tracking_input1', 'wp_tracking_input2'])

        # load fly up control pid gain
        path = os.path.dirname(__file__)
        filepath = str(path) + '/flyup_pid_gain/%s.yml' % tello1_ns
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

        desired_Ql1 = userdata.wp_tracking_input1
        desired_Ql2 = userdata.wp_tracking_input2
        preErr1_x = 0.0
        preErr1_y = 0.0
        preErr1_z = 0.0
        sumErr1_x = 0.0
        sumErr1_y = 0.0
        sumErr1_z = 0.0
        preErr2_x = 0.0
        preErr2_y = 0.0
        preErr2_z = 0.0
        sumErr2_x = 0.0
        sumErr2_y = 0.0
        sumErr2_z = 0.0
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            err1 = desired_Ql1 - sub1.Ql
            err2 = desired_Ql2 - sub2.Ql

            if (err1.dot(err1) < 0.01) and (err2.dot(err2) < 0.01): # distance < 5 cm
                # pubs.util_smach('WP_TRACK', 'WP_ASSIGN')
                return 'wp_tracking_success'

            ###### PID for agent 1#######
            sumErr1_x = sumErr1_x + err1[0]
            sumErr1_y = sumErr1_y + err1[1]
            sumErr1_z = sumErr1_z + err1[2]
            dErr1_x = err1[0] - preErr1_x
            dErr1_y = err1[1] - preErr1_y
            dErr1_z = err1[2] - preErr1_z
            ux = self.Kp_x * err1[0] + self.Ki_x * sumErr1_x + self.Kd_x * dErr1_x
            uy = self.Kp_y * err1[1] + self.Ki_y * sumErr1_y + self.Kd_y * dErr1_y
            uz = self.Kp_z * err1[2] + self.Ki_z * sumErr1_z + self.Kd_z * dErr1_z
            u1 = np.array([ux, uy, uz])
            preErr1_x = err1[0]
            preErr1_y = err1[1]
            preErr1_z = err1[2]
            ###### PID #######

            ###### PID for agent 2#######
            sumErr2_x = sumErr2_x + err2[0]
            sumErr2_y = sumErr2_y + err2[1]
            sumErr2_z = sumErr2_z + err2[2]
            dErr2_x = err2[0] - preErr2_x
            dErr2_y = err2[1] - preErr2_y
            dErr2_z = err2[2] - preErr2_z
            ux = self.Kp_x * err2[0] + self.Ki_x * sumErr2_x + self.Kd_x * dErr2_x
            uy = self.Kp_y * err2[1] + self.Ki_y * sumErr2_y + self.Kd_y * dErr2_y
            uz = self.Kp_z * err2[2] + self.Ki_z * sumErr2_z + self.Kd_z * dErr2_z
            u2 = np.array([ux, uy, uz])
            preErr2_x = err2[0]
            preErr2_y = err2[1]
            preErr2_z = err2[2]
            ###### PID #######
            
            u1 = sub1.bRc.dot(sub1.cRm.dot(sub1.mRl.dot(u1)))
            u2 = sub2.bRc.dot(sub2.cRm.dot(sub2.mRl.dot(u2)))
            pub1.util_cmd(u1[0], u1[1], u1[2], 0)
            pub2.util_cmd(u2[0], u2[1], u2[2], 0)
            rate.sleep()
        
        return 'wp_tracking_error'

class sLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['land_finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LAND')

        pub1.util_hover()
        pub2.util_hover()
        rospy.sleep(0.5)
        pub1.util_land()
        pub2.util_land()
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
            self.sm_flyup_control.userdata.desired_Ql1 = None
            self.sm_flyup_control.userdata.desired_Ql2 = None
            with self.sm_flyup_control:
                smach.StateMachine.add('WP_ASSIGN', sWpAssign(), 
                    transitions={'wp_assigned':'WP_TRACK', 'wp_assign_finish':'flyup_control_finish'},
                    remapping={'wp_assign_output1':'desired_Ql1', 'wp_assign_output2':'desired_Ql2'})
                smach.StateMachine.add('WP_TRACK', sWpTracking(), 
                    transitions={'wp_tracking_success':'WP_ASSIGN', 'wp_tracking_error': 'flyup_control_error'},
                    remapping={'wp_tracking_input1':'desired_Ql1', 'wp_tracking_input2':'desired_Ql2'})

            smach.StateMachine.add('FLYUP_CONTROL', self.sm_flyup_control, 
                transitions={'flyup_control_finish':'LAND', 'flyup_control_error':'control_finish'})
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

def shutdown_hook():
    print("************in shutdown hook*************")
    pub1.util_hover()
    pub2.util_hover()
    rospy.sleep(0.5)
    pub1.util_land()
    pub2.util_land()

if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)

    # task setting
    tello1_ns = rospy.get_param('~tello1_ns', "tello_601")
    ap1_id = int(rospy.get_param('~ap1_id', "tello_601"))
    tello2_ns = rospy.get_param('~tello2_ns', "tello_601")
    ap2_id = int(rospy.get_param('~ap2_id', "tello_601"))

    sub1 = Subs(tello1_ns)
    sub2 = Subs(tello2_ns)
    pub1 = Pubs(tello1_ns)    
    pub2 = Pubs(tello2_ns)
    pub_sm = PubSm()

    rospy.on_shutdown(shutdown_hook)
    
    Control()
    rospy.spin()