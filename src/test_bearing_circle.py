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
    
    return A, dist, vec

def main():
    import sys
    global image_hsv, pixel # so we can use it in mouse callback
    indices = [129,130,131,132,133,134,471,472,473,474,475,615,616,617,618,619,1158,1159,1160,1161,1162,1419,1420,1421,1422,1423]

    for i in range(0,26):
        index = indices[i]


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
        c = c[c[:,0]!=0,:]
        c = c[c[:,0]!=959,:]
        c = c[c[:,1]!=0,:]
        c = c[c[:,1]!=719,:]
        sh = c.shape
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

        D = np.hstack((cx**2+cy**2, cx, cy, np.ones((sh[0], 1))))
        del c
        del cx
        del cy

        # find ellipse and eigen
        F = D.T@D
        del D
        A, dist, vec = elpMtxQ(F)
        
        # bearing in {C}
        bc = 0.02*dist*vec
        if bc[2] < 0:
            bc = -bc        

        x = np.linspace(-2.0, 2.0, 1000)
        y = np.linspace(-2.0, 2.0, 1000)
        X, Y = np.meshgrid(x,y)
        F = A[0]*X**2 + A[0]*Y**2 + A[1]*X + A[2]*Y + A[3]
        plt.contour(X,Y,F,[0])
        plt.xlim([-0.3, 0.2])
        plt.ylim([-0.65, -0.15])
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        plt.xlabel("$x_c (m)$",  fontsize=18)
        plt.ylabel("$y_c (m)$",  fontsize=18)

        # print(bc)
        plt.savefig("/home/kuei/Documents/records/202305/0522/0521_Test6_rerun/fitting_circle/%d.svg"%index, bbox_inches='tight')
        plt.close()
        
        
        # cv2.imshow("frame", frame)
        # cv2.waitKey(0)
        # plt.show()


if __name__=='__main__':
    main()