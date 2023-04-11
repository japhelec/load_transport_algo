#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from load_transport.msg import position_msg
from scipy.linalg import eig

class Bearing():
    def __init__(self):      
        self.tello_ns = rospy.get_param('~tello_ns', "tello_601")
        self.br = CvBridge()
        self.mtx = np.linalg.inv(np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]]))

        # [sub]
        self.sub_image = rospy.Subscriber("/%s/camera/compressed/compressed" % self.tello_ns, CompressedImage, self.cb_image, queue_size = 1)

        # [pub]
        self.pub_bearing = rospy.Publisher('/%s/bearing/local' % self.tello_ns, position_msg, queue_size=1)

    def cb_image(self, img):
        # cvBridge
        frame = self.br.compressed_imgmsg_to_cv2(img)

        # convert to hsv colorspace
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        # C is orange, D is green, E is blue
        # C track green, D track blue, E track orange
        if self.tello_ns == "tello_C":
            lower_bound = np.array([63, 77, 14])	 
            upper_bound = np.array([86, 156, 201])
        elif self.tello_ns == "tello_D":
            lower_bound = np.array([107, 133, -4])
            upper_bound = np.array([132, 243, 230])
        elif self.tello_ns == "tello_E":
            lower_bound = np.array([3, 150, 135])	 
            upper_bound = np.array([26, 265, 265])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # contour points
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            contour = max(contours, key = len)
            area = cv2.contourArea(contour)

            
            if area > 100:
                cv2.drawContours(frame, contour, -1, (0,255,0), 1)

                # formulate D matrix
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

                # find ellipse 
                F = D.T@D
                del D
                
                w, vr = eig(F)
                A = vr[:, np.argmin(w)]

                B = A[1]
                C = A[2]
                D = A[3]
                E = A[4]
                F = A[5]
                A = A[0]

                xc = (B*E-2*C*D)/(4*A*C-B*B)
                yc = (D*B-2*A*E)/(4*A*C-B*B)

                # find bearing
                t = np.array([xc, yc, 1])
                t = self.mtx@(t)

                # publish bearing
                msg = position_msg()
                msg.header.stamp = rospy.get_rostime()
                msg.position = t
                self.pub_bearing.publish(msg)

        cv2.imshow(self.tello_ns, frame)
        cv2.waitKey(1)

def main():
    rospy.init_node('bearing_perception', anonymous=True)
    Bearing()
    rospy.spin()

if __name__ == '__main__':
    main()