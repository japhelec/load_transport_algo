#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from load_transport.msg import cRm_msg, Mc_msg
from hardware import Marker

class Perception():
    def __init__(self):      
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")

        # [ camera ]
        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        self.br = CvBridge()
        self.sub_image = rospy.Subscriber("/%s/camera/image_raw" % self.tello_ns, Image, self.cb_marker_perception, queue_size = 1)
        self.pub_cRm = rospy.Publisher('/%s/cRm' % self.tello_ns, cRm_msg, queue_size=1)
        self.pub_Mc = rospy.Publisher('/%s/Mc' % self.tello_ns, Mc_msg, queue_size=1)

    def cb_marker_perception(self, img_raw):
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
            # print(rvec)
            # c0m0_c = tvec[0][0] 
            # cRm, jacob = cv2.Rodrigues(rvec) 

            # P_m = self.getP_m(id)
            # P_c = c0m0_c + cRm.dot(P_m)
            # # P_b = self.bRc.dot(P_c) + np.array([0,0.05,0])
            # P_b = self.bRc.dot(P_c)

            msg = cRm_msg()
            msg.header.stamp = rospy.get_rostime()
            msg.marker_id = int(id)
            msg.rvec = rvec[0][0]
            self.pub_cRm.publish(msg)

            msg = Mc_msg()
            msg.header.stamp = rospy.get_rostime()
            msg.marker_id = int(id)
            msg.tvec = tvec[0][0]
            self.pub_Mc.publish(msg)


def main():
    rospy.init_node('marker_perception', anonymous=True)
    Perception()
    rospy.spin()


if __name__ == '__main__':
    main()