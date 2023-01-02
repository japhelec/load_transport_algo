#!/usr/bin/env python3

import sys
import yaml
import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

DEBUG = False

class MarkerDetect():
    def __init__(self): 
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")
        # self.tello_ns = "tello_A"

        # [ camera ]
        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        # [ cv Bridge ]
        self.br = CvBridge()
        self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb_image_compressed, queue_size = 1)


        markersX = 6
        markersY = 5
        markerLength = 3.15
        markerSeparation = 1.05
        dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)


    def cb_image_compressed(self, cp_img):
        ## cvBridge
        img = self.br.compressed_imgmsg_to_cv2(cp_img, desired_encoding="bgr8")

        ## gray
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ## detect
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        aruco.drawDetectedMarkers(img, corners, ids)

        ## DEBUG (show markers)
        # cv2.imshow(self.tello_ns,img)
        # key = cv2.waitKey(1)
        
        ## estimation
        if ids is not None:
            rvec = np.array([[1,0,0],[0,1,0],[0,0,1]], dtype='f')
            tvec = np.array([0,0,0], dtype='f')
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.mtx, self.dist, rvec, tvec)  # from C to M
            # print(tvec)
            # print("here")

def main():
    rospy.init_node('marker_detect', anonymous=True)
    MarkerDetect()
    rospy.spin()


if __name__ == '__main__':
    main()