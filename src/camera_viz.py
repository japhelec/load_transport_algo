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
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")
        self.img_type = rospy.get_param('~img_type', "image_raw")
        
        # [ ROS publisher subscriber ]
        if self.img_type == "image_raw":
            self.sub_image = rospy.Subscriber("/%s/camera/image_raw" % self.tello_ns, Image, self.cb_image_raw, queue_size = 1)
        else:
            self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb_image_compressed, queue_size = 1)

    def cb_image_raw(self, img_raw):
        ## cvBridge
        frame = self.br.imgmsg_to_cv2(img_raw)

        ## gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow(self.tello_ns,frame)
        key = cv2.waitKey(1)

    def cb_image_compressed(self, img):
        ## cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img, desired_encoding="bgr8")

        ## gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow(self.tello_ns,frame)
        key = cv2.waitKey(1)

def main():
    rospy.init_node('camera_viz', anonymous=True)
    MarkerDetect()
    rospy.spin()


if __name__ == '__main__':
    main()