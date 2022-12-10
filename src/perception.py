#!/usr/bin/env python3

import sys
import yaml
import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from load_transport.msg import P_b_msg, payload_msg


#############################################
#coordinate frame
# c: camera frame
# p: payload frame
# m: captured marker frame
# b: Q frame
# P: ap on payload
#############################################


class Payload():
    length = 0.41 #m
    width = 0.675 #m

class Marker():
    length = 0.05 #m

class Perception():
    def __init__(self, tello_ns):      
        # [ camera ]
        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        # [ cam to body ]
        filepath = '/home/kuei/catkin_ws/src/load_transport/src/camera_calib/%s.yml' % "tello_601"
        with open(filepath, 'r') as f:
            data = yaml.load(f)

        self.bTc = np.array(data['T'])   # from tello attach point to camera
        self.bRc = np.array(data['R'])   # from tello attach point to camera

        self.br = CvBridge()
        self.sub_image = rospy.Subscriber("/%s/camera/image_raw" % tello_ns, Image, self.cb_image, queue_size = 1)
        self.pub_P_b = rospy.Publisher('P_b', P_b_msg, queue_size=1)
        self.pub_payload = rospy.Publisher('payload', payload_msg, queue_size=1)

        self.ap_id = 1

    def cb_image(self, img_raw):
        ## cvBridge
        frame = self.br.imgmsg_to_cv2(img_raw)

        ## gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ## detect
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        
        ## estimation
        if ids is not None:
            # any marker is ok
            id = ids[0][0]
            corner = corners[0]
            
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, Marker.length, self.mtx, self.dist)
            c0m0_c = tvec[0][0] 
            cRm, jacob = cv2.Rodrigues(rvec) 

            P_m = self.getP_m(id)
            P_c = c0m0_c + cRm.dot(P_m)
            P_b = self.bRc.dot(P_c) + np.array([0,0.05,0])

            msg = P_b_msg()
            msg.header.stamp = rospy.get_rostime()
            msg.P_b = P_b
            self.pub_P_b.publish(msg)

            msg = payload_msg()
            msg.header.stamp = rospy.get_rostime()
            print("==============")
            print(np.dot(self.bRc, np.dot(cRm, self.get_mRp(id))))
            msg.bRp = np.dot(self.bRc, np.dot(cRm, self.get_mRp(id))).reshape([9,1])
            self.pub_payload.publish(msg)

    def getP_p(self):      
        if self.ap_id == 0 or self.ap_id == 6 or self.ap_id == 7:
            x = Payload.width/2
        elif self.ap_id == 1 or self.ap_id == 5:
            x = 0
        else:
            x = -Payload.width/2

        if self.ap_id == 0 or self.ap_id == 1 or self.ap_id == 2:
            y = Payload.length/2
        elif self.ap_id == 3 or self.ap_id == 7:
            y = 0
        else:
            y = -Payload.length/2

        return np.array([x, y, 0])

    def getM0_p(self, marker_id):      
        if marker_id == 0 or marker_id == 3:
            x = Payload.width/2 - Marker.length/2
        else:
            x = -Payload.width/2 + Marker.length/2

        if marker_id == 0 or marker_id == 1:
            y = Payload.length/2 - Marker.length/2
        else:
            y = -Payload.length/2 + Marker.length/2

        return np.array([x, y, 0])

    def get_pRm(self, marker_id):
        angle = np.pi/2*marker_id
        pRm = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        return pRm

    def get_mRp(self, marker_id):
        angle = -np.pi/2*marker_id
        mRp = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        return mRp
        
    def getP_m(self, marker_id):
        P_p = self.getP_p()
        M0_p = self.getM0_p(marker_id)

        pRm = self.get_pRm(marker_id)

        P_m = np.linalg.inv(pRm).dot(P_p - M0_p)
        return P_m 



def main():
    rospy.init_node('marker_detect', anonymous=True)
    Perception(tello_ns)
    rospy.spin()


if __name__ == '__main__':
    tello_ns = sys.argv[1]
    main()