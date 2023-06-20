#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.linalg import eig

def main():
    import sys
    global image_hsv, pixel # so we can use it in mouse callback
    pic = "pic10"

    image_src = cv2.imread("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s.png"%pic)  # pick.py my.png
    if image_src is None:
        print ("the image read is None............")
        return
    cv2.imshow("bgr",image_src)

    ## NEW ##
    cv2.namedWindow('hsv')

    # now click into the hsv img , and look at values:
    image_hsv = cv2.cvtColor(image_src,cv2.COLOR_BGR2HSV)
    cv2.imwrite("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s_hsv.png"%pic, image_hsv)
    cv2.imshow("hsv",image_hsv)


    cv2.namedWindow('mask')
    # # orange
    # lower_bound = np.array([3, 150, 135])	 
    # upper_bound = np.array([26, 265, 265])
    # green
    lower_bound = np.array([63, 77, 14])	 
    upper_bound = np.array([86, 156, 201])
    # # purple
    # lower_bound = np.array([115, 153, 1])	 
    # upper_bound = np.array([146, 242, 152])
    

    # find the colors within the boundaries
    mask = cv2.inRange(image_hsv, lower_bound, upper_bound)
    cv2.imwrite("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s_mask.png"%pic, mask)
    cv2.imshow("mask",mask)

    # contour points
    cv2.namedWindow('contour')
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    
    contour = max(contours, key = len)
    image_contour = image_src.copy()
    cv2.drawContours(image_contour, contour, -1, (0,255,0), 5)
    cv2.imshow("contour",image_contour)
    cv2.imwrite("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s_contour.png"%pic, image_contour)

    # ellipse
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
    cv2.circle(image_contour, (int(xc), int(yc)), 8, (0, 0, 255), -1)
    # cv2.drawContours(image_src, contour, -1, (0,255,0), 1)
    cv2.imshow("ellipse",image_contour)
    cv2.imwrite("/home/kuei/catkin_ws/src/load_transport/src/bearing_detect/%s_ellipse.png"%pic, image_contour)



    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()