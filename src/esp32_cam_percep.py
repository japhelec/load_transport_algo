#!/usr/bin/env python3

import sys
import yaml
import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from load_transport.msg import position_msg, rotation_msg

#############################################
#coordinate frame
# c: camera frame
# l: payload frame
# m: captured marker frame
# b: Q frame
# P: ap on payload
#############################################

class Marker():
    length = 0.05 #m

class Perception():
    def __init__(self):
        with open('/home/kuei/catkin_ws/src/load_transport/src/camera_calib/esp32_intrinsic.yml', 'r') as f:
            data = yaml.load(f, Loader=yaml.UnsafeLoader)
        
        self.mtx = np.array(data['mtx'])
        self.dist = np.array(data['dist'])

        self.br = CvBridge()
        self.sub_image = rospy.Subscriber("/esp32/image_raw", Image, self.cb_marker_perception, queue_size = 1)
        self.pub_p1_position = rospy.Publisher('/p1/position', position_msg, queue_size=1)
        self.pub_payload_rotation = rospy.Publisher('/payload/rotation', rotation_msg, queue_size=1)

    def cb_marker_perception(self, img_raw):
        ## cvBridge
        frame = self.br.imgmsg_to_cv2(img_raw)

        ## detect
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame,aruco_dict,parameters=parameters)
        
        ## estimation
        if ids is not None:
            # any marker is ok
            id = ids[0][0]
            corner = corners[0]
            
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, Marker.length, self.mtx, self.dist)
            
            # print("=====================")
            # print(tvec[0][0])
            # R, jacob = cv2.Rodrigues(rvec[0][0])
            # print(R)

            msg = position_msg()
            msg.header.stamp = rospy.get_rostime()
            msg.position = tvec[0][0]
            self.pub_p1_position.publish(msg)

            msg = rotation_msg()
            msg.header.stamp = rospy.get_rostime()
            msg.rotation = rvec[0][0]
            self.pub_payload_rotation.publish(msg)


def main():
    rospy.init_node('esp32_percep', anonymous=True)
    Perception()
    rospy.spin()


if __name__ == '__main__':
    main()