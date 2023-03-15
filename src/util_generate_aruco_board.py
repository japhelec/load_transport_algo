#!/usr/bin/env python3

import cv2
from cv2 import aruco

markersX = 2
markersY = 2
markerLength = 0.05
markerSeparation = 0.02
dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)

board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary, 0)
img = board.draw((500, 500))

# cv2.imshow("aruco board",img)
# key = cv2.waitKey(0)

cv2.imwrite('./aruco_board/1st_.png',img)