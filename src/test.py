#!/usr/bin/env python3

import sys
import yaml
import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point

class MarkerDetect():
    def __init__(self):      
        # [ cv Bridge ]
        self.br = CvBridge()
        
        # [ ROS publisher subscriber ]
        self.sub_image = rospy.Subscriber("/esp32/compressed/compressed", CompressedImage, self.cb_image_compressed, queue_size = 1)

    def cb_image_compressed(self, img_raw):
        ## cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img_raw, desired_encoding="bgr8")
        # print(frame)

        ## gray
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow("hello",frame)
        key = cv2.waitKey(1)

def main():
    rospy.init_node('camera_viz', anonymous=True)
    MarkerDetect()
    rospy.spin()


if __name__ == '__main__':
    main()