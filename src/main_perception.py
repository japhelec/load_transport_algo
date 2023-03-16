#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from load_transport.msg import cRm_msg, Mc_msg, position_msg
from hardware import Marker, Payload, Drone
from util_rot import RotM

class Perception():
    def __init__(self):      
        self.br = CvBridge()
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")
        self.id = Drone.ns2id(self.tello_ns)

        # [ camera ]
        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        # [board]
        markersX = 6
        markersY = 5
        markerLength = 0.0315
        markerSeparation = 0.0105
        dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)

        # [kal]
        self.Kal_x = KalmanFilter()
        self.Kal_y = KalmanFilter()
        self.Kal_z = KalmanFilter()
        self.Mc = None

        # [sub]
        self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb_marker_perception, queue_size = 1)

        # [pub]
        self.pub_cRm_raw = rospy.Publisher('/%s/cRm/raw' % self.tello_ns, cRm_msg, queue_size=1)
        self.pub_cRm_filtered = rospy.Publisher('/%s/cRm/filtered' % self.tello_ns, cRm_msg, queue_size=1)
        self.pub_Mc_raw = rospy.Publisher('/%s/Mc/raw' % self.tello_ns, Mc_msg, queue_size=1)
        self.pub_Mc_filtered = rospy.Publisher('/%s/Mc/filtered' % self.tello_ns, Mc_msg, queue_size=1)
        self.pub_Ql_raw = rospy.Publisher('/%s/Ql/raw' % self.tello_ns, position_msg, queue_size=1)
        self.pub_Ql_filtered = rospy.Publisher('/%s/Ql/filtered' % self.tello_ns, position_msg, queue_size=1)
        
        self.filter_pub()

    def filter_pub(self):
        rate = rospy.Rate(15)
        
        while not rospy.is_shutdown():
            if self.Kal_x.access() is not None:
                x = self.Kal_x.predict()
                y = self.Kal_y.predict()
                z = self.Kal_z.predict()

                cRm = RotM.toRotm(x, y, z)
                rvec, jacob = cv2.Rodrigues(cRm)
                tvec = self.Mc

                self.publish_filtered_cRm(rvec) # accept row
                self.publish_filtered_Mc(tvec) # accept row
                self.publish_filtered_Ql(rvec, tvec)

            rate.sleep()

    def cb_marker_perception(self, img_raw):
        ## cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img_raw)

        ## gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ## detect
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        
        ## estimation
        if ids is not None:
            rvec = np.array([[1,0,0],[0,1,0],[0,0,1]], dtype='f')
            tvec = np.array([0,0,0], dtype='f')

            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.mtx, self.dist, rvec, tvec)  # from C to M
            self.Mc = tvec
        
            self.publish_raw_cRm(rvec.T[0]) # accept row
            self.publish_raw_Mc(tvec) # accept row
            self.publish_raw_Ql(rvec, tvec)

            # filter init / update
            cRm, jacob = cv2.Rodrigues(rvec) 
            x, y, z = RotM.Euler_zxy(cRm)
            if self.Kal_x.access() is None:
                self.Kal_x.init(x)
                self.Kal_y.init(y)
                self.Kal_z.init(z)
            else:
                self.Kal_x.update(x)
                self.Kal_y.update(y)
                self.Kal_z.update(z)

    def publish_raw_cRm(self, rvec):
        msg_cRm = cRm_msg()
        msg_cRm.header.stamp = rospy.get_rostime()
        msg_cRm.marker_id = int(self.id)
        msg_cRm.rvec = rvec
        self.pub_cRm_raw.publish(msg_cRm)

    def publish_raw_Mc(self, tvec):
        msg_Mc = Mc_msg()
        msg_Mc.header.stamp = rospy.get_rostime()
        msg_Mc.marker_id = int(self.id)
        msg_Mc.tvec = tvec
        self.pub_Mc_raw.publish(msg_Mc)

    def publish_raw_Ql(self, rvec, tvec):
        msg_Ql = position_msg()
        msg_Ql.header.stamp = rospy.get_rostime()
        cRm, jacob = cv2.Rodrigues(rvec) 
        Mc = tvec
        Qm = -(cRm.T).dot(Mc) + cRm.T.dot(np.array([0,0.05,0]))
        Ql = Payload.mRl().dot(Qm)
        msg_Ql.position = Ql
        self.pub_Ql_raw.publish(msg_Ql)

    def publish_filtered_cRm(self, rvec):
        msg_cRm = cRm_msg()
        msg_cRm.header.stamp = rospy.get_rostime()
        msg_cRm.marker_id = int(self.id)
        msg_cRm.rvec = rvec
        self.pub_cRm_filtered.publish(msg_cRm)

    def publish_filtered_Mc(self, tvec):
        msg_Mc = Mc_msg()
        msg_Mc.header.stamp = rospy.get_rostime()
        msg_Mc.marker_id = int(self.id)
        msg_Mc.tvec = tvec
        self.pub_Mc_filtered.publish(msg_Mc)

    def publish_filtered_Ql(self, rvec, tvec):
        msg_Ql = position_msg()
        msg_Ql.header.stamp = rospy.get_rostime()
        cRm, jacob = cv2.Rodrigues(rvec) 
        Mc = tvec
        Qm = -(cRm.T).dot(Mc) + cRm.T.dot(np.array([0,0.05,0]))
        Ql = Payload.mRl().dot(Qm)
        msg_Ql.position = Ql
        self.pub_Ql_filtered.publish(msg_Ql)


class KalmanFilter:
    def __init__(self):
        dt = 1.0/15.0
        self.x = None
        self.P = None
        
        self.F = np.array([[1, dt], [0, 1]])
        self.Q = np.array([
            [dt*dt*dt*dt/4, dt*dt*dt/2],
            [dt*dt*dt/2, dt*dt]
        ])*0.7

        self.H = np.array([1,0])
        self.R = 0.3

    def access(self):
        if self.x is None:
            return None
        else:
            return self.x[0]

    def init(self, x0):
        self.x = np.array([x0, 0])
        self.P = np.array([[1,0], [0,1]])

    def predict(self):
        self.x = self.F.dot(self.x)

        if self.x[0] > np.pi:
            self.x[0] = self.flipping(self.x[0])

        elif self.x[0] < -np.pi:
            self.x[0] = self.flipping(self.x[0])

        self.P = np.matmul(self.F, np.matmul(self.P, self.F.T)) + self.Q

        return self.access()

    def update(self, z):

        aux = self.access()

        if (np.absolute(aux - z) > np.pi):
            z = self.flipping(z)

        y = z - self.H.dot(self.x)
        rhs = 1 / (self.H.dot(self.P.dot(self.H)) + self.R)
        K = self.P.dot(self.H)*rhs
        self.x = self.x + K*y  
        self.P = np.matmul((np.identity(2)-np.outer(K, self.H)), self.P)

    def flipping(self, val):
        val_abs = np.abs(val)
        sgn = np.sign(val)

        return -1*sgn*(2*np.pi-val_abs)


def main():
    rospy.init_node('marker_perception', anonymous=True)
    Perception()
    rospy.spin()


if __name__ == '__main__':
    main()