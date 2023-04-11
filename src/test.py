#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from load_transport.msg import cRm_msg, Mc_msg, position_msg
from hardware import Marker, Payload
from scipy.sparse.linalg import eigs
from scipy.linalg import eig

class Perception():
    def __init__(self):      
        self.tello_ns = "tello_E"

        # [ camera ]
        self.dist = np.array(([[-0.016272, 0.093492, 0.000093, 0.002999, 0]]))
        self.mtx = np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]])

        self.br = CvBridge()
        self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb, queue_size = 1)


        self.preQl = None
        self.preT = None

    def cb(self, img_raw):
        ## cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img_raw)

        # convert to hsv colorspace
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        # ========ping pong========
        # lower_bound = np.array([3, 150, 135])	 
        # upper_bound = np.array([26, 265, 265])
        # # ========blue========
        # lower_bound = np.array([107, 133, -4])	 
        # upper_bound = np.array([132, 243, 211])
        # # ========red========
        # lower_bound = np.array([-10, 93, 36])	 
        # upper_bound = np.array([189, 265, 292])
        # # ========purple========
        # lower_bound = np.array([115, 153, 1])	 
        # upper_bound = np.array([146, 242, 152])
        # ========green========
        lower_bound = np.array([63, 77, 14])	 
        upper_bound = np.array([86, 156, 201])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # contour points
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contour = max(contours, key = len)

        cv2.drawContours(frame, contour, -1, (0,255,0), 1)

        # formulate D matrix
        print("==============")
        sh = contour.shape
        c = contour.reshape(sh[0], sh[2])
        del contour
        cx = c[:,0]
        cx = cx[:, None]
        cy = c[:,1]
        cy = cy[:, None]
        
        D = np.hstack((cx**2,cx*cy, cy**2, c, np.ones((sh[0], 1))))
        del c
        del cx
        del cy
        F = D.T@D
        del D
        
        w, vr = eig(F)
        A = vr[:, np.argmin(w)]

        # ===================
        # verify ellipse center
        B = A[1]
        C = A[2]
        D = A[3]
        E = A[4]
        F = A[5]
        A = A[0]

        xc = (B*E-2*C*D)/(4*A*C-B*B)
        yc = (D*B-2*A*E)/(4*A*C-B*B)
        # print(xc)
        # print(yc)

        aux = np.array([xc, yc, 1])

        t = np.linalg.inv(self.mtx)@(aux)
        print(t)
        
        # ===================


        # ===================
        # a1 = A[0]
        # a2 = A[2]
        # a3 = A[1]/2
        # a4 = A[3]/2
        # a5 = A[4]/2
        # a6 = A[5]
        
        # Q = np.array([[a1, a3, a4],[a3, a2, a5], [a4, a5, a6]])
        # w, vr = eig(Q)

        # sorted_indexes = np.argsort(w)
        # w = w[sorted_indexes]
        # vr = vr[:,sorted_indexes]
        # # # print(w)

        # l2 = w[0]
        # l0 = w[1]
        # l1 = w[2]

        # q2 = vr[:, 0]
        # q0 = vr[:, 1]
        # q1 = vr[:, 2]

        # n = np.sqrt((l1-l0)/(l1-l2))*q0 + np.sqrt((l0-l2)/(l1-l2))*q2
        # print(n)

        # # print(q2)

        # t = l2*np.sqrt((l1-l0)/(l1-l2))*q0 + l1*np.sqrt((l0-l2)/(l1-l2))*q2
        # t = 40*t/np.sqrt(-l1*l2)
        # # ===================
        # # print(t)

        cv2.imshow("stream", frame)
        cv2.waitKey(1)
        

def main():
    rospy.init_node('marker_perception', anonymous=True)
    Perception()
    rospy.spin()


if __name__ == '__main__':
    main()