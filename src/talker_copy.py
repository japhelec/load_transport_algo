#!/usr/bin/env python3

import sys
import time
import rospy
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Talker():
    def __init__(self):      
        self.odom = np.array([0,0,0])
        self.marker = np.array([0,0,0])

        self.pub_motor_on = rospy.Publisher('manual_takeoff', Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.cb_odom, queue_size = 1)
        self.sub_marker = rospy.Subscriber('marker', Point, self.cb_marker, queue_size = 1)

        rospy.sleep(2.0) # warm up for publishing

        # experiment
        self.case_motor_on_and_land()

    def case_motor_on_and_land(self):
        self.util_motor_on()
        rospy.sleep(2.0)
        self.util_land()

    def util_wait(self, duration):
        rospy.sleep(duration)

    def util_motor_on(self):
        self.pub_motor_on.publish()

    def util_hover(self):
        msg = Twist()
        self.pub_cmd_vel.publish(msg)

    def util_land(self):
        self.pub_land.publish()

    def util_flyup(self, thrust, duration):
        msg = Twist()
        msg.linear.z = thrust
        self.pub_cmd_vel.publish(msg)
        rospy.sleep(duration)

        msg = Twist()
        self.pub_cmd_vel.publish(msg)

    def cb_odom(self, odom):
        pos = odom.pose.pose.position
        self.odom = np.array([pos.x, pos.y, pos.z])

    def cb_marker(self, marker):
        self.marker = np.array([marker.x, marker.y, marker.z])


def main():
    rospy.init_node('talker', anonymous=True)
    Talker()
    rospy.spin()


if __name__ == '__main__':
    main()