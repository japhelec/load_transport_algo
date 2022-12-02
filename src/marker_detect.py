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

DEBUG = False

class MarkerDetect():
    def __init__(self, tello_ns):      
        # [ camera ]
        # dist=np.array(([[0.02622212, -0.55220608, -0.0034407, 0.00321558, 1.89103285]]))
        # mtx=np.array([[901.57301941  , 0.      ,   477.52938592],
        # [  0.       ,  907.36572961, 355.00994502],
        # [  0.,           0.,           1.        ]])
        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        # [ camera & attach point]
        filepath = '/home/kuei/catkin_ws/src/load_transport/src/camera_calib/%s.yml' % tello_ns
        with open(filepath, 'r') as f:
            data = yaml.load(f)

        self.T = data['T']
        self.R = data['R']

        # [ cv Bridge ]
        self.br = CvBridge()
        
        # [ ROS publisher subscriber ]
        self.sub_image = rospy.Subscriber("/%s/camera/image_raw" % tello_ns, Image, self.cb_image, queue_size = 1)
        self.pub_marker = rospy.Publisher('marker', Point, queue_size=1)

    def cb_image(self, img_raw):
        ## cvBridge
        frame = self.br.imgmsg_to_cv2(img_raw)

        ## gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if DEBUG:
            cv2.imshow("frame",frame)
            key = cv2.waitKey(1)

        ## detect
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

        ## estimation
        if ids is not None:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)
            R, jacob = cv2.Rodrigues(rvec) # C to M
            corner_M = np.array([0.025, 0.025, 0])
            corner_C = tvec[0][0] + np.dot(R, corner_M)
            ap = self.T + np.dot(self.R, corner_C)
            
            msg = Point()
            msg.x = ap[0]
            msg.y = ap[1]
            msg.z = ap[2]
            self.pub_marker.publish(msg)

def main():
    rospy.init_node('marker_detect', anonymous=True)
    MarkerDetect(tello_ns)
    rospy.spin()


if __name__ == '__main__':
    tello_ns = sys.argv[1]
    main()