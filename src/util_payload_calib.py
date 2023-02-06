#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from load_transport.msg import position_msg, rotation_msg

class Marker():
    length = 0.05 #m

class Perception():
    def __init__(self):      
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")

        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        self.br = CvBridge()
        self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb_marker_perception, queue_size = 1)

    def cb_marker_perception(self, img_raw):
        ## cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img_raw)

        ## detect
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame,aruco_dict,parameters=parameters)
        
        ## estimation
        if ids is not None:
            sz = ids.size
            if (sz != 8):
                return

            print("========================")
            for i in range(0,sz):
                id = ids[i][0]
                print("id: %d", id)
                
                corner = corners[i]
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, Marker.length, self.mtx, self.dist)                
                print(tvec[0][0])
                
            # # R, jacob = cv2.Rodrigues(rvec[0][0])
            # # print(R)

            # msg = position_msg()
            # msg.header.stamp = rospy.get_rostime()
            # msg.position = tvec[0][0]
            # self.pub_p2_position.publish(msg)

            # msg = rotation_msg()
            # msg.header.stamp = rospy.get_rostime()
            # msg.rotation = rvec[0][0]
            # self.pub_payload_rotation.publish(msg)


def main():
    rospy.init_node('tello_percep', anonymous=True)
    Perception()
    rospy.spin()


if __name__ == '__main__':
    main()