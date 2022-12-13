#!/usr/bin/env python3

import sys
import yaml
import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class MarkerDetect():
    def __init__(self, tello_ns):      
        # [ cv Bridge ]
        self.br = CvBridge()
        self.tello_ns = tello_ns
        
        # [ ROS publisher subscriber ]
        self.sub_image = rospy.Subscriber("/%s/camera/image_raw" % tello_ns, Image, self.cb_image, queue_size = 1)

    def cb_image(self, img_raw):
        ## cvBridge
        frame = self.br.imgmsg_to_cv2(img_raw)

        ## gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow(tello_ns,frame)
        key = cv2.waitKey(1)

def main():
    rospy.init_node('camera_viz', anonymous=True)
    MarkerDetect(tello_ns)
    rospy.spin()


if __name__ == '__main__':
    tello_ns = sys.argv[1]
    main()