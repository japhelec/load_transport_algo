#!/usr/bin/env python3

import sys
import time
import rospy
import numpy as np
from std_msgs.msg import Empty
from load_transport.msg import marker_msg
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

class Talker():
    def __init__(self):      
        self.drone_id = int(rospy.get_param('~drone_id', 1))

        self.Q_i = np.array([0,0,0])
        self.iRb = np.array([[0,0,0], [0,0,0], [0,0,0]])
        self.Pi_b = np.array([0,0,0])
        self.bRl = np.array([[0,0,0],[0,0,0],[0,0,0]])

        self.pub_motor_on = rospy.Publisher('manual_takeoff', Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.cb_odom, queue_size = 1)
        self.sub_marker = rospy.Subscriber('marker', marker_msg, self.cb_marker, queue_size = 1)

        rospy.sleep(5.0) # warm up for publishing

        # experiment
        # self.case_motorOn_flyUp_Land()
        self.case_stabilize_attitude()

    def case_motorOn_and_land(self):
        self.util_motor_on()
        self.util_wait(2.0)
        self.util_land()

    def case_motorOn_flyUp_Land(self):
        self.util_motor_on()
        self.util_wait(5.0)
        self.util_flyup(1)
        print("================================================")
        self.util_wait(5)
        self.util_hover()
        # self.util_wait(0.5)

        print("================================================")


        starttime = time.time()
        rate = rospy.Rate(10) 

        while not rospy.is_shutdown():

            curret = time.time()
            sofar = curret - starttime

            if (sofar < 5) :
                rate.sleep()
            else:
                break
        print("out of loop")

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

    def case_stabilize_attitude(self):
        self.util_motor_on()
        self.util_wait(5.0)

        # fly up
        self.util_flyup(1)
        self.util_wait(5)

        self.util_hover()
        self.util_wait(0.5)
        
        # control
        self.control_alignXY_constZ()
        
        self.util_hover()
        self.util_hover()
        self.util_hover()
        self.util_hover()
        self.util_wait(1)

        self.util_land()
        self.util_land()
        self.util_land()
        self.util_land()

    
    def control_alignXY_constZ(self, duration):
        Kp_x = 3
        Kp_y = 3
        Kp_z = 3

        starttime = time.time()
        rate = rospy.Rate(10) 

        while not rospy.is_shutdown():

            curret = time.time()
            sofar = curret - starttime

            if (sofar < duration) :
                iRl = self.iRb.dot(self.bRl)

                # print("===========")
                rot_axis = -(iRl - iRl.T)
                # print(rot_axis)
                rot_axis = np.array([rot_axis[2,1], rot_axis[0,2], rot_axis[1, 0]])

                Pi_i = iRl.dot(Payload.getPi_l(self.drone_id))
                
                u = np.cross(rot_axis, Pi_i)
                self.util_cmd(u[0]*Kp_x, u[1]*Kp_y, u[2]*Kp_z)

                # print(rot_axis)
                # print(Pi_i)
                # print(u)

                rate.sleep()
            else:
                break

        print("================================================")
        print("================================================")
        print("================================================")
        print("================================================")
        print("================================================")
        print("out of loop")
        print("================================================")
        print("================================================")
        print("================================================")
        print("================================================")
        print("================================================")
        

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

        # print(self.iRb)

    def cb_marker(self, marker):
        self.Pi_b = np.array(marker.Pi_b)
        self.bRl = np.array(marker.bRl).reshape([3,3])

        # print("===marker====")
        # print(self.Pi_b)
        # print(self.iRb.dot(self.bRl))

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