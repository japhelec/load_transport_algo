#!/usr/bin/env python3

# package
import rospy
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
            if subs.marker_id is not None:
                return 'flyup_open_finish'
            rate.sleep()
        return 'flyup_open_error'

# class sFlyupNav1(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['flyup_nav1_finish'])
#         print("**************************")
#         rospy.loginfo('Executing state FLYUP_NAV1')
#         print("**************************")

#     def execute(self, userdata):

#         while 

#         return 'up_open_finish'

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
                transitions={'flyup_open_finish':'LAND', 'flyup_open_error':'control_finish'})
            smach.StateMachine.add('LAND', sLand(), 
                transitions={'land_finish':'control_finish'})
            # smach.StateMachine.add('UP_OPEN', sUpOpen(), 
            #     transitions={'up_open_finish':'control_finish'})
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
