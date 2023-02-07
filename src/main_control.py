#!/usr/bin/env python3

# package
import rospy
import smach
import numpy as np
import cv2

# message
from std_msgs.msg import Empty
from load_transport.msg import cRm_msg, Mc_msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix

# Hardward Config
from hardware import Payload, Drone


class Control():
    def __init__(self):
        self.tello_ns = "tello_601"

        self.pub_motor_on = rospy.Publisher('/%s/manual_takeoff' % self.tello_ns, Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/%s/cmd_vel' % self.tello_ns, Twist, queue_size=1)
        self.pub_land = rospy.Publisher('/%s/land' % self.tello_ns, Empty, queue_size=1)

        self.Q_i = np.array([0,0,0])
        self.iRb = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.marker_id = None
        self.Mc = np.array([0,0,0])
        self.cRm = np.array([[1,0,0],[0,1,0],[0,0,1]])
        
        self.sub_odom = rospy.Subscriber('/%s/odom' % self.tello_ns, Odometry, self.cb_odom, queue_size = 1)
        self.sub_cRm = rospy.Subscriber('/%s/cRm' % self.tello_ns, cRm_msg, self.cb_cRm, queue_size = 1)
        self.sub_Mc = rospy.Subscriber('/%s/Mc' % self.tello_ns, Mc_msg, self.cb_Mc, queue_size = 1)

        # self.sm

        # rospy.sleep(5.0) # warm up for publishing

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

    def cb_Mc(self, data):
        self.marker_id = data.marker_id
        self.Mc = np.array(data.tvec)


def main():
    rospy.init_node('control', anonymous=True)
    Control()
    rospy.spin()

if __name__ == '__main__':
    main()