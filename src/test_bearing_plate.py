#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.linalg import eig

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
        return None, None
    
    return w, vr

def main():
    import sys
    global image_hsv, pixel # so we can use it in mouse callback
    pic = "pic1"

    frame = cv2.imread("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s.png"%pic)  # pick.py my.png
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

    D = np.hstack((cx**2,cx*cy, cy**2, cx, cy, np.ones((sh[0], 1))))
    del c
    del cx
    del cy

    # find ellipse and eigen
    F = D.T@D
    del D
    w, vr = elpMtxQ(F)

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

    print(bc)     

    cv2.imshow("frame", frame)
    cv2.waitKey(0)

if __name__=='__main__':
    main()