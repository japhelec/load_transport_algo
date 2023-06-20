#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.linalg import eig
import matplotlib.pyplot as plt

mtx = np.linalg.inv(np.array([[929.562627  , 0.      ,   487.474037],
        [  0.       ,  928.604856, 361.165223],
        [  0.,           0.,           1.        ]]))

def elpMtxQ(F):
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
        return None, None, None
    
    return [Q[0,0], Q[0,1]*2, Q[1,1], Q[0,2]*2, Q[1,2]*2, Q[2,2]], w, vr

def main():
    import sys
    global image_hsv, pixel # so we can use it in mouse callback
    indices = [129,130,131,132,133,134,471,472,473,474,475,615,616,617,618,619,1158,1159,1160,1161,1162,1419,1420,1421,1422,1423]

    for i in range(0,26):
        index = indices[i]
        # index = 25

        # frame = cv2.imread("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s.png"%index)  # indexk.py my.png
        frame = cv2.imread("/home/kuei/Documents/records/202305/0522/0521_Test6_rerun/raw/%d.png"%index)  # indexk.py my.png
        if frame is None:
            print ("the image read is None............")
            return

        # convert to hsv colorspace
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Green color
        # C is orange, D is green, E is blue
        # C track green, D track blue, E track orange
        lower_bound = np.array([3, 150, 135])	 
        upper_bound = np.array([26, 265, 265])

        # find the colors within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # contour points
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contour = max(contours, key = len)
        cv2.drawContours(frame, contour, -1, (0,255,0), 1)

        # contour points back to camera frame
        sh = contour.shape
        c = contour.reshape(sh[0], sh[2])
        del contour
        
        cx = c[:,0]
        cx = cx[:, None]
        cy = c[:,1]
        cy = cy[:, None]

        aux = np.hstack((cx,cy, np.ones((sh[0], 1))))
        aux = mtx@(aux.T)
        aux = aux.T

        # formulate D matrix
        cx = aux[:,0]
        cx = cx[:, None]
        cy = aux[:,1]
        cy = cy[:, None]
        del aux
        plt.scatter(cx, cy)

        D = np.hstack((cx**2,cx*cy, cy**2, cx, cy, np.ones((sh[0], 1))))
        del c
        del cx
        del cy

        # find ellipse and eigen
        F = D.T@D
        del D
        A, w, vr = elpMtxQ(F)

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

        x = np.linspace(-2.0, 2.0, 1000)
        y = np.linspace(-2.0, 2.0, 1000)
        X, Y = np.meshgrid(x,y)
        # print(bc)
        F = A[0]*X**2 + +A[1]*X*Y + A[2]*Y**2 + A[3]*X + A[4]*Y + A[5]
        plt.contour(X,Y,F,[0])
        plt.xlim([-0.3, 0.2])
        plt.ylim([-0.65, -0.15])
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        plt.xlabel("$x_c (m)$",  fontsize=18)
        plt.ylabel("$y_c (m)$",  fontsize=18)

        # print(bc)
        plt.savefig("/home/kuei/Documents/records/202305/0522/0521_Test6_rerun/fitting_ellipse/%d.svg"%index, bbox_inches='tight')
        plt.close()

if __name__=='__main__':
    main()