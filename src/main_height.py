#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from tello_driver.msg import TelloStatus

from load_transport.msg import position_msg
from hardware import Marker, Payload, Drone
from util_rot import RotM

class Height_Filter():
    def __init__(self):      
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")

        # [kal]
        self.Kal = KalmanFilter()
        self.h = None

        # [sub]
        self.sub_imu = rospy.Subscriber("/%s/imu" % self.tello_ns, Imu, self.cb_imu, queue_size = 1)
        self.sub_tof = rospy.Subscriber("/%s/status" % self.tello_ns, TelloStatus, self.cb_tof, queue_size = 1)

        # [pub]
        self.pub_h_filtered = rospy.Publisher('/%s/height/filtered' % self.tello_ns, position_msg, queue_size=1)
        
        self.filter_pub()

    def filter_pub(self):
        rate = rospy.Rate(15)
        
        while not rospy.is_shutdown():
            h = self.Kal.predict()

            msg_h = position_msg()
            msg_h.header.stamp = rospy.get_rostime()
            msg_h.position = np.array([0,0,h])
            self.pub_h_filtered.publish(msg_h)

            rate.sleep()

    def cb_imu(self, msg):
        z = msg.linear_acceleration.z
        self.Kal.update_imu(z*(-10)-9.81)

    def cb_tof(self, msg):
        self.Kal.update_tof(msg.height_m)

class KalmanFilter:
    def __init__(self):
        # Q_gain = float(rospy.get_param('~height_Q', "0.7"))
        # R_gain = float(rospy.get_param('~height_R', "0.3"))
        
        dt = 1.0/15.0
        self.x = np.array([0,0,0])
        self.P = np.array([[1,0,0], [0,1,0],[0,0,1]])
        
        self.F = np.array([
            [1, dt, dt*dt/2.0], 
            [0, 1, dt], 
            [0,0,1]
        ])

        self.Q = np.array([
            [1, 0, 0],
            [0, 0.1, 0],
            [0, 0, 0.01]
        ])

        self.H_tof = np.array([1,0,0])
        self.H_imu = np.array([0,0,1])
        self.R_tof = 10
        self.R_imu = 0.1

    def access(self):
        return self.x[0]

    def predict(self):
        self.x = self.F.dot(self.x)
        self.P = np.matmul(self.F, np.matmul(self.P, self.F.T)) + self.Q

        return self.access()

    def update_imu(self, az):
        y = az - self.H_imu.dot(self.x)
        rhs = 1 / (self.H_imu.dot(self.P.dot(self.H_imu)) + self.R_imu)
        K = self.P.dot(self.H_imu)*rhs
        self.x = self.x + K*y  
        self.P = np.matmul((np.identity(3)-np.outer(K, self.H_imu)), self.P)

    def update_tof(self, h):
        y = h - self.H_tof.dot(self.x)
        rhs = 1 / (self.H_tof.dot(self.P.dot(self.H_tof)) + self.R_tof)
        K = self.P.dot(self.H_tof)*rhs
        self.x = self.x + K*y  
        self.P = np.matmul((np.identity(3)-np.outer(K, self.H_tof)), self.P)

def main():
    rospy.init_node('height_filter', anonymous=True)
    Height_Filter()
    rospy.spin()

if __name__ == '__main__':
    main()