#!/usr/bin/env python3

import sys
import time
import rospy
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix

class Talker():
    def __init__(self):      
        self.odom_pos = np.array([0,0,0])
        self.odom_orien = np.array([[0,0,0], [0,0,0], [0,0,0]])
        self.marker = np.array([0,0,0])

        self.pub_motor_on = rospy.Publisher('manual_takeoff', Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.cb_odom, queue_size = 1)
        self.sub_marker = rospy.Subscriber('marker', Point, self.cb_marker, queue_size = 1)

        rospy.sleep(6.0) # warm up for publishing

        # experiment
        # self.case_flyUp_rotate_Land()

    def case_motorOn_and_land(self):
        self.util_motor_on()
        self.util_wait(2.0)
        self.util_land()

    def case_motorOn_flyUp_Land(self):
        self.util_motor_on()
        self.util_wait(2.0)
        self.util_flyup(0.5)
        self.util_wait(5)
        self.util_hover()
        self.util_wait(0.5)
        self.util_land()

    def case_flyUp_rotate_Land(self):
        self.util_motor_on()
        self.util_wait(2.0)

        # fly up
        self.util_flyup(1)
        self.util_wait(1.5)

        self.util_hover()
        self.util_wait(0.5)

        # back ward
        self.util_cmd(0,-1,0,0)
        self.util_wait(1.0)

        self.util_hover()
        self.util_wait(0.5)

        # forward
        self.util_cmd(0,1,0,0)
        self.util_wait(1.0)

        self.util_hover()
        self.util_wait(0.5)

        # rotate
        self.util_cmd(0,0,0,1)
        self.util_wait(0.6)

        self.util_hover()
        self.util_wait(1.2)

        # rotate
        self.util_cmd(0,0,0,-1)
        self.util_wait(0.6)

        self.util_hover()
        self.util_wait(0.5)

        self.util_land()

    def case_alignXY_constZ(self):
        # self.util_motor_on()
        # self.util_wait(2.0)
        # self.util_land()
        # print("case_alignXY_trackZ")
        self.control_alignXY_constZ(5, 5)
    
    def control_alignXY_constZ(self, duration, z):
        Kp_x = 1
        Kp_y = 1
        Kp_z = 1

        starttime = time.time()
        rate = rospy.Rate(10) 

        while not rospy.is_shutdown():

            curret = time.time()
            sofar = curret - starttime

            if (sofar < duration) :
                rate.sleep()
            else:
                break

    def util_cmd(self, x, y, z, yaw):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = yaw
        self.pub_cmd_vel.publish(msg)

    def util_wait(self, duration):
        rospy.sleep(duration)

    def util_motor_on(self):
        self.pub_motor_on.publish()

    def util_hover(self):
        self.util_cmd(0,0,0,0)

    def util_land(self):
        self.pub_land.publish()

    def util_flyup(self, thrust):
        self.util_cmd(0,0,thrust,0)

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

        self.odom_pos = np.array([pos.y, pos.x, -pos.z])
        self.odom_orien = rotm

        # print(self.odom_orien)

    def cb_marker(self, marker):
        self.marker = np.array([marker.x, marker.y, marker.z])

        # print("===marker====")
        # print(self.marker)

    def test_loop_duration(self, duration):
        # continuously looping for a duration
        starttime = time.time()
        rate = rospy.Rate(10) 

        while not rospy.is_shutdown():
            curret = time.time()
            sofar = curret - starttime
            print(sofar)

            if (sofar < duration) :
                rate.sleep()
            else:
                break

def main():
    rospy.init_node('talker', anonymous=True)
    Talker()
    rospy.spin()


if __name__ == '__main__':
    main()