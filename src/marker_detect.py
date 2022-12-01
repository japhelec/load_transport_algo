#!/usr/bin/env python3

import sys
import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv2 import aruco
import yaml

DEBUG = False


def callback(img_raw):
    ## cvBridge
    frame = br.imgmsg_to_cv2(img_raw)

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
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        ap = T + np.dot(R, tvec)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/%s/camera/image_raw" % tello_ns, Image, callback, queue_size = 1)
    rospy.spin()


if __name__ == '__main__':
    # [old self-calibrated camera config]
    # dist=np.array(([[0.02622212, -0.55220608, -0.0034407, 0.00321558, 1.89103285]]))

    # mtx=np.array([[901.57301941  , 0.      ,   477.52938592],
    # [  0.       ,  907.36572961, 355.00994502],
    # [  0.,           0.,           1.        ]])

    # [calibrated by tello_driver source code]
    dist=np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
    mtx=np.array([[929.562627  , 0.      ,   487.474037],
    [  0.       ,  928.604856, 361.165223],
    [  0.,           0.,           1.        ]])

    tello_ns = sys.argv[1]
    filepath = '/home/kuei/catkin_ws/src/load_transport/src/camera_calib/%s.yml' % tello_ns
    with open(filepath, 'r') as f:
        data = yaml.load(f)
    
    T = data['T']
    R = data['R']

    br = CvBridge()
    main()