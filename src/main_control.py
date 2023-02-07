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
from load_transport.msg import cRm_msg, Mc_msg, position_msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix

# Hardward Config
from hardware import Payload, Drone


class sWarmup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['warmup_finish'])
        print("**************************")
        rospy.loginfo('Executing state WARMUP')
        print("**************************")

    def execute(self, userdata):
        rospy.sleep(5.0)
        pubs.util_motor_on()
        rospy.sleep(5.0)
        return 'warmup_finish'

class sFlyupOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_open_finish', 'flyup_open_error'])
        print("**************************")
        rospy.loginfo('Executing state FLYUP_OPEN')
        print("**************************")

    def execute(self, userdata):
        pubs.util_cmd(0, 0, 0.8, 0)

        rate = rospy.Rate(50) # check image comes in?
        while not rospy.is_shutdown():
            if subs.Ql is not None:
                return 'flyup_open_finish'
            rate.sleep()
        return 'flyup_open_error'

class sFlyupControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flyup_control_finish', 'flyup_control_error'])
        print("**************************")
        rospy.loginfo('Executing state FLYUP_CONTROL')
        print("**************************")
        self.counter = 0

    def execute(self, userdata):
        ##### TO FIX #####
        desired_Ql = Payload.Pl(6) + np.array([0, -0.1, 0.7])
        ##### TO FIX #####

        path = os.path.dirname(__file__)
        filepath = str(path) + '/flyup_pid_gain/%s.yml' % tello_ns
        with open(filepath, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        Kp_x = data['Kp_x']
        Kp_y = data['Kp_y']
        Kp_z = data['Kp_z']
        Kd_x = data['Kd_x']
        Kd_y = data['Kd_y']
        Kd_z = data['Kd_z']
        Ki_x = data['Ki_x']
        Ki_y = data['Ki_y']
        Ki_z = data['Ki_z']
        
        preErr_x = 0.0
        preErr_y = 0.0
        preErr_z = 0.0
        sumErr_x = 0.0
        sumErr_y = 0.0
        sumErr_z = 0.0
        
        rate = rospy.Rate(15) 
        while not rospy.is_shutdown():
            err = desired_Ql - subs.Ql

            ###### PID #######
            sumErr_x = sumErr_x + err[0]
            sumErr_y = sumErr_y + err[1]
            sumErr_z = sumErr_z + err[2]
            dErr_x = err[0] - preErr_x
            dErr_y = err[1] - preErr_y
            dErr_z = err[2] - preErr_z
            ux = Kp_x * err[0] + Ki_x * sumErr_x + Kd_x * dErr_x
            uy = Kp_y * err[1] + Ki_y * sumErr_y + Kd_y * dErr_y
            uz = Kp_z * err[2] + Ki_z * sumErr_z + Kd_z * dErr_z
            u = np.array([ux, uy, uz])
            preErr_x = err[0]
            preErr_y = err[1]
            preErr_z = err[2]
            ###### PID #######
            
            u = subs.bRc.dot(subs.cRm.dot(subs.mRl.dot(u)))
            pubs.util_cmd(u[0], u[1], u[2], 0)
            rate.sleep()
        
        return 'flyup_control_error'

class sLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['land_finish'])
        print("**************************")
        rospy.loginfo('Executing state LAND')
        print("**************************")

    def execute(self, userdata):
        pubs.util_hover()
        rospy.sleep(0.5)
        pubs.util_land()
        return 'land_finish'

class sFake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fake_finish'])
        print("**************************")
        rospy.loginfo('Executing state FAKE')
        print("**************************")

    def execute(self, userdata):
        return 'fake_finish'

class Control():
    def __init__(self):        
        self.sm = smach.StateMachine(outcomes=['control_finish'])
        with self.sm:
            smach.StateMachine.add('WARMUP', sWarmup(), 
                transitions={'warmup_finish':'FLYUP_OPEN'})
            smach.StateMachine.add('FLYUP_OPEN', sFlyupOpen(), 
                transitions={'flyup_open_finish':'FLYUP_CONTROL', 'flyup_open_error':'control_finish'})
            smach.StateMachine.add('FLYUP_CONTROL', sFlyupControl(), 
                transitions={'flyup_control_finish':'LAND', 'flyup_control_error':'control_finish'})
            smach.StateMachine.add('LAND', sLand(), 
                transitions={'land_finish':'control_finish'})
        smach_thread = threading.Thread(target=self.sm.execute, daemon = True)
        smach_thread.start()

class Pubs():
    def __init__(self):
        # publisher
        self.pub_motor_on = rospy.Publisher('/%s/manual_takeoff' % tello_ns, Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/%s/cmd_vel' % tello_ns, Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/%s/land' % tello_ns, Empty, queue_size=1)

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
    tello_ns = "tello_601"
    ap_id = 2
    subs = Subs()
    pubs = Pubs()    
    
    Control()
    rospy.spin()
