#!/usr/bin/env python3

# input is bag of compressed image topic of chessboard

import cv2 as cv
import numpy as np
import yaml
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


# configuration
bagfile = '/home/kuei/Documents/records/20230111/esp32_cam_calibration/chessboard_2023-01-11-09-52-18.bag'
image_topic = '/esp32/compressed/compressed'


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objp = objp*2.3 #(cm)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

num = 0

with open('./camera_calib/esp32_intrinsic.yml', 'w') as f:
    # d = {'A':'a', 'B':{'C':'c', 'D':'d', 'E':'e'}}
    # d = np.array([1,2,3])
    # d = [[1,2,3], [4,5,6]]
    # yaml.dump(d.tolist(), f, default_flow_style=False)
    br = CvBridge()
    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        frame = br.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # cv2.imshow("hello",frame)
        # key = cv2.waitKey(20)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(frame, (9,6), None)
        if ret == True:
            num += 1

            if num %10 == 0:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(frame,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
            # Draw and display the corners
            # cv.drawChessboardCorners(frame, (9,6), corners2, ret)
            # cv.imshow('img', frame)
            # cv.waitKey(1)

            if num >= 850:
                break

    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frame.shape[::-1], None, None)
    print("====mtx====")
    print(mtx)
    print("====dist====")
    print(dist)

    d = {'mtx':mtx, 'dist': dist}
    yaml.dump(d, f, default_flow_style=False)

    bag.close()