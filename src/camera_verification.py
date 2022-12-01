#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv2 import aruco


def callback(img_raw):
    ## cvBridge
    frame = br.imgmsg_to_cv2(img_raw)

    ## gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("frame",frame)
    key = cv2.waitKey(1)

    ## detect
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    ## estimation
    if ids is not None:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        marker_C = tvec[0][0] # C to M
        R_M2C, jacob = cv2.Rodrigues(rvec) # C to M
        marker_A = T_A2C+np.dot(R_A2C, marker_C)
        print(marker_A)
        

def listener():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/tello_C/camera/image_raw", Image, callback, queue_size = 1)
    rospy.spin()


if __name__ == '__main__':
    ##### config of camera params
    # dist=np.array(([[0.02622212, -0.55220608, -0.0034407, 0.00321558, 1.89103285]]))

    # mtx=np.array([[901.57301941  , 0.      ,   477.52938592],
    # [  0.       ,  907.36572961, 355.00994502],
    # [  0.,           0.,           1.        ]])
    dist=np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))

    mtx=np.array([[929.562627  , 0.      ,   487.474037],
    [  0.       ,  928.604856, 361.165223],
    [  0.,           0.,           1.        ]])

    T_A2C = np.array([-0.01338165, 0.05092815, 0.01320737]) # AP to M
    R_A2C = np.array([[0.99957758, -0.0131693, 0.02590818], [-0.00680008, -0.97267753, -0.23206074], [0.02825638, 0.23178653, -0.9723562]]) # AP to M
    

    br = CvBridge()  
    listener()