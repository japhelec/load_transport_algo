#!/usr/bin/env python3

import sys
import time
import rospy
import numpy as np
from std_msgs.msg import Empty
from load_transport.msg import P_b_msg, payload_msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix

class Payload():
    length = 0.2095
    width = 0.3375

    @classmethod
    def getPi_l(cls, id):
        if id == 0 or id == 3:
            W_sign = 1
        else:
            W_sign = -1
        
        if id == 0 or id == 1:
            L_sign = 1
        else:
            L_sign = -1

        Pi_l = np.array([cls.width*W_sign, cls.length*L_sign, 0])
        return Pi_l

class Action():
    def __init__(self):      
        self.Q_i = np.array([0,0,0])
        self.iRb = np.array([[1,0,0], [0,1,0], [0,0,1]])
        self.P_b = np.array([0,0,0])
        self.iRl = np.array([[1,0,0],[0,1,0],[0,0,1]])

        self.pub_motor_on = rospy.Publisher('manual_takeoff', Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.cb_odom, queue_size = 1)
        self.sub_payload = rospy.Subscriber('payload', payload_msg, self.cb_payload, queue_size = 1)
        self.sub_P_b = rospy.Subscriber('P_b', P_b_msg, self.cb_P_b, queue_size = 1)

        rospy.sleep(5.0) # warm up for publishing

        # experiment
        # self.case_motorOn_flyUp_Land()
        # self.case_stabilize_attitude()
        self.case_single_lift_middle()

    def case_motorOn_and_land(self):
        self.util_motor_on()
        self.util_wait(2.0)
        self.util_land()

    def case_motorOn_flyUp_Land(self):
        self.util_motor_on()
        self.util_wait(5.0)
        self.util_flyup(2)
        
        self.util_wait(6)
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

    def case_single_lift_middle(self):
        print("==================================")
        print("in case")
        print("==================================")
        self.util_motor_on()
        self.util_wait(5.0)

        # fly up
        self.util_flyup(1)
        self.util_wait(5)
        
        # control
        self.control_single_lift(15)
        
        self.util_hover()
        self.util_wait(1)

        self.util_land()

    def control_single_lift(self, duration):
        Kp_x = 2
        Kp_y = 2
        Kp_z = 3

        starttime = time.time()
        rate = rospy.Rate(15) 

        while not rospy.is_shutdown():

            curret = time.time()
            sofar = curret - starttime

            if (sofar < duration) :
                z_axis_of_iRl = self.iRl.dot(np.array([0,0,1]))
                current_angle = np.arccos(z_axis_of_iRl.dot(np.array([0,0,1]))) # rad
                desire_angle = np.pi/6
                angle_err = desire_angle - current_angle

                ux = Kp_x * self.P_b[0]
                uy = Kp_y * self.P_b[1]
                uz = Kp_z * angle_err

                self.util_cmd(ux, uy, uz, 0)

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

        self.Q_i = np.array([pos.y, pos.x, -pos.z])
        self.iRb = rotm

        # print("====================")
        # print(self.iRb)

    def cb_payload(self, payload):
        bRl = np.array(payload.bRl).reshape([3,3])
        self.iRl = self.iRb.dot(bRl)

        # print("====================")
        # print(self.iRl)

    def cb_P_b(self, msg):
        self.P_b = np.array(msg.P_b)
        # print("====================")
        # print(self.P_b)        

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
    rospy.init_node('action', anonymous=True)
    Action()
    rospy.spin()


if __name__ == '__main__':
    main()