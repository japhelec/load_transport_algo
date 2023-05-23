#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from load_transport.msg import position_msg
from nav_msgs.msg import Odometry
from scipy.linalg import eig
from hardware import Drone
from tf.transformations import quaternion_matrix


class Bearing():
    def __init__(self):      
        self.tello_ns = rospy.get_param('~tello_ns', "tello_D")
        self.br = CvBridge()
        self.mtx = np.linalg.inv(np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]]))
        self.iRb = None

        # [sub]
        self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb_image, queue_size = 1)
        self.sub_odom = rospy.Subscriber('/%s/odom' % self.tello_ns, Odometry, self.cb_odom, queue_size = 1)

        # [pub]
        self.pub_bearing_local_plate = rospy.Publisher('/%s/bearing/local/plate' % self.tello_ns, position_msg, queue_size=1)
        self.pub_bearing_local_circle = rospy.Publisher('/%s/bearing/local/circle' % self.tello_ns, position_msg, queue_size=1)
        self.pub_bearing_global_plate = rospy.Publisher('/%s/bearing/global/plate' % self.tello_ns, position_msg, queue_size=1)
        self.pub_bearing_global_circle = rospy.Publisher('/%s/bearing/global/circle' % self.tello_ns, position_msg, queue_size=1)

    def cb_image(self, img):
        # cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img)
        iRb = np.copy(self.iRb)

        # convert to hsv colorspace
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        # C is orange, D is green, E is blue
        # C track green, D track blue, E track orange
        if self.tello_ns == "tello_C":
            lower_bound = np.array([115, 153, 1])	 
            upper_bound = np.array([146, 242, 152])
        elif self.tello_ns == "tello_D":
            # lower_bound = np.array([107, 133, -4])
            # upper_bound = np.array([132, 243, 230])
            lower_bound = np.array([3, 150, 135])	 
            upper_bound = np.array([26, 265, 265])
        elif self.tello_ns == "tello_A":
            # lower_bound = np.array([63, 77, 14])	 
            # upper_bound = np.array([86, 156, 201])
            # === sunlight ===
            lower_bound = np.array([60, 70, 14])	 
            upper_bound = np.array([88, 130, 201])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # contour points
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            contour = max(contours, key = len)
            area = cv2.contourArea(contour)
            
            if area > 350:
                cv2.drawContours(frame, contour, -1, (0,255,0), 1)

                self.bearing_estimation_plate(contour, iRb)
                self.bearing_estimation_circle(contour, iRb)

        # cv2.imshow(self.tello_ns, frame)
        # cv2.waitKey(1)

    def bearing_estimation_plate(self, contour, iRb):
        # contour points back to camera frame
        sh = contour.shape
        c = contour.reshape(sh[0], sh[2])
        
        cx = c[:,0]
        cx = cx[:, None]
        cy = c[:,1]
        cy = cy[:, None]

        reproj = np.hstack((cx,cy, np.ones((sh[0], 1))))
        reproj = self.mtx@(reproj.T)
        reproj = reproj.T

        # formulate D matrix
        cx = reproj[:,0]
        cx = cx[:, None]
        cy = reproj[:,1]
        cy = cy[:, None]

        D = np.hstack((cx**2,cx*cy, cy**2, cx, cy, np.ones((sh[0], 1))))
        del c
        del cx
        del cy

        # find ellipse and eigen
        F = D.T@D
        del D
        w, vr = self.elpMtxQ_plate(F)

        if w is None:
            return

        sorted_indexes = np.argsort(w)
        w = w[sorted_indexes]
        vr = vr[:,sorted_indexes]

        l2 = w[0]
        l0 = w[1]
        l1 = w[2]

        q2 = vr[:, 0]
        q0 = vr[:, 1]
        q1 = vr[:, 2]

        # bearing in {C}
        bc = l2*np.sqrt((l1-l0)/(l1-l2))*q1 + l1*np.sqrt((l0-l2)/(l1-l2))*q2
        bc = 0.02*bc/np.sqrt(-l1*l2)
        bc = bc.real
        if bc[2] < 0:
            bc = -bc

        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = bc
        self.pub_bearing_local_plate.publish(msg)

        # bearing in {W}
        bw = (iRb)@(Drone.bRc)@(Drone.camTilt)@bc

        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = bw
        self.pub_bearing_global_plate.publish(msg)

    def bearing_estimation_circle(self, contour, iRb):
        # contour points back to camera frame
        sh = contour.shape
        c = contour.reshape(sh[0], sh[2])
        c = c[c[:,0]!=0,:]
        c = c[c[:,0]!=959,:]
        c = c[c[:,1]!=0,:]
        c = c[c[:,1]!=719,:]
        sh = c.shape
        
        cx = c[:,0]
        cx = cx[:, None]
        cy = c[:,1]
        cy = cy[:, None]

        reproj = np.hstack((cx,cy, np.ones((sh[0], 1))))
        reproj = self.mtx@(reproj.T)
        reproj = reproj.T

        # formulate D matrix
        cx = reproj[:,0]
        cx = cx[:, None]
        cy = reproj[:,1]
        cy = cy[:, None]

        D = np.hstack((cx**2+cy**2, cx, cy, np.ones((sh[0], 1))))
        del cx
        del cy

        # find ellipse and eigen
        F = D.T@D
        del D
        dist, vec = self.elpMtxQ_circle(F)
        
        # bearing in {C}
        bc = 0.02*dist*vec
        if bc[2] < 0:
            bc = -bc     

        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = bc
        self.pub_bearing_local_circle.publish(msg)

        # bearing in {W}
        bw = (iRb)@(Drone.bRc)@(Drone.camTilt)@bc

        msg = position_msg()
        msg.header.stamp = rospy.get_rostime()
        msg.position = bw
        self.pub_bearing_global_circle.publish(msg)
                

    def cb_odom(self, odom):
        orien = odom.pose.pose.orientation
        
        rotm = quaternion_matrix([orien.x, orien.y, orien.z, orien.w])  # x, y, z, w;   quaternion is w + xi + yj + zk
        rotm = np.array(rotm)
        rotm = rotm[0:3, 0:3]
        Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # rotate along x 180
        Rz_p = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # rotate along z 90
        Rz_n = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) # rotate along z -90
        rotm = Rz_p.dot(Rx.dot(rotm.dot(Rx.dot(Rz_n)))) # Rz_p -> Rx -> rotm -> Rx -> Rz_n

        self.iRb = rotm

    def elpMtxQ_plate(self, F):
        w, vr = eig(F)
        A = vr[:, np.argmin(w)]

        B = A[1]
        C = A[2]
        D = A[3]
        E = A[4]
        F = A[5]
        A = A[0]

        Q = np.array([
            [A, B/2, D/2],
            [B/2, C, E/2],
            [D/2, E/2, F]
        ])

        # check proper conic
        w, vr = eig(Q)
        if ((0 < w).sum()) != 2:
            Q = -Q
            w, vr = eig(Q)

        # check parabola or ellipse
        A = Q[0,0]
        B = Q[0,1]
        C = Q[1,1]

        if (A*C-B*B) < 0:
            return None, None
        
        return w, vr

    def elpMtxQ_circle(self, F):
        w, vr = eig(F)
        A = vr[:, np.argmin(w)]

        a = A[0]
        d = A[1]
        e = A[2]
        f = A[3]

        Q = np.array([
            [a, 0, d/2],
            [0, a, e/2],
            [d/2, e/2, f]
        ])

        w, vr = eig(Q)
        sorted_indexes = np.argsort(w)
        w = w[sorted_indexes]
        vr = vr[:,sorted_indexes]

        negative_count = (w < 0).sum()
        positive_count = (w > 0).sum()

        if negative_count == 1:
            vec = vr[:,0]
            dist = np.sqrt( np.absolute((w[1]+w[2])/(2*w[0])) + 1)

        if positive_count == 1:
            vec = vr[:,2]
            dist = np.sqrt( np.absolute((w[1]+w[0])/(2*w[2])) + 1)
        
        return dist, vec

def main():
    rospy.init_node('bearing_perception', anonymous=True)
    Bearing()
    rospy.spin()

if __name__ == '__main__':
    main()