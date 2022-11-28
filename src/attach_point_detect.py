#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv2 import aruco


def callback(ros_data):

    print("image received")

    ## cvBridge
    frame = br.imgmsg_to_cv2(ros_data)

    ## gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("frame",frame)
    key = cv2.waitKey(1)


    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()

    ## detect
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    if ids is not None:

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.175, mtx, dist)

        print(tvec)
    
    #     (rvec-tvec).any()

    #     for i in range(rvec.shape[0]):
    #         aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
    #         aruco.drawDetectedMarkers(frame, corners)
    
    #     cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
    # else:
    #     cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # cv2.imshow("frame",frame)
    # key = cv2.waitKey(2)

def listener():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/tello_601/camera/image_raw", Image, callback, queue_size = 1)
    rospy.spin()


if __name__ == '__main__':
    ##### config of camera params
    dist=np.array(([[0.02622212, -0.55220608, -0.0034407, 0.00321558, 1.89103285]]))

    mtx=np.array([[901.57301941  , 0.      ,   477.52938592],
    [  0.       ,  907.36572961, 355.00994502],
    [  0.,           0.,           1.        ]])

    font = cv2.FONT_HERSHEY_SIMPLEX

    br = CvBridge()  
    listener()