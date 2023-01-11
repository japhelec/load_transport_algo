#!/usr/bin/env python3

import cv2
from cv2 import aruco

markersX = 6
markersY = 5
markerLength = 0.15
markerSeparation = 0.05
dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)

board = cv2.aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)
img = board.draw((2700, 900))

# cv2.imshow("aruco board",img)
# key = cv2.waitKey(0)

cv2.imwrite('./aruco_board/image.png',img)