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

    print("=================")
    print("%f, %f, %f, %f, %f, %f" % (A, B, C, D, E, F))
    print(A*C-B*B/4)

    # eigen
    w, vr = eig(Q)
    if ((0 < w).sum()) != 2:
        w, vr = eig(-Q)
    
    return w, vr

def main():
    
    for i in range(1,261):
    # for i in range(1, 2):
        frame = cv2.imread("/home/kuei/src/temp/0523/raw/%d.png"%i)  # pick.py my.png
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
        area = cv2.contourArea(contour)
            
                # print(contour)
        cv2.drawContours(frame, contour, -1, (0,255,0), 1)
        cv2.imwrite('/home/kuei/src/temp/0523/contour/%d.png'%i,frame)           

if __name__=='__main__':
    main()