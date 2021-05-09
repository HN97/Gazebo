#! /usr/bin/python3

import rospy
import mavros
import mavros_msgs.msg

import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os
import imutils

markerSize = 0.2
axisLength = 1

img = cv2.imread("./marker10.jpg")
# img = imutils.resize(img, width=800)
# camera intristic parameters
cameraMatrix = np.array([277.191356, 0.0, 160.5, 0.0, 277.191356, 120.5, 0.0, 0.0, 1.0]).reshape(3,3)
distCoeffs = np.array([0.0] * 5)

# Check for camera calibration data
if not os.path.exists('./CameraCalibration.txt'):
    print("You need to calibrate the camera you are using.")
    exit()
else:
    f = open('./CameraCalibration.txt', 'rb')
    f.close()

arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParams = aruco.DetectorParameters_create()
# Detect the markers in the image
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
(corners, ids, rejected) = aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

# # verify *at least* one ArUco marker was detected
# if len(corners) > 0:
# 	# flatten the ArUco IDs list
# 	ids = ids.flatten()
# 	# loop over the detected ArUCo corners
# 	for (markerCorner, markerID) in zip(corners, ids):
# 		# extract the marker corners (which are always returned in
# 		# top-left, top-right, bottom-right, and bottom-left order)
# 		corners = markerCorner.reshape((4, 2))
# 		(topLeft, topRight, bottomRight, bottomLeft) = corners
# 		# convert each of the (x, y)-coordinate pairs to integers
# 		topRight = (int(topRight[0]), int(topRight[1]))
# 		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
# 		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
# 		topLeft = (int(topLeft[0]), int(topLeft[1]))


# # draw the bounding box of the ArUCo detection
# cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
# cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
# cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
# cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

# # compute and draw the center (x, y)-coordinates of the ArUco
# # marker
# cX = int((topLeft[0] + bottomRight[0]) / 2.0)
# cY = int((topLeft[1] + bottomRight[1]) / 2.0)
# cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
rotation_vectors, translation_vectors, ret = aruco.estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs)
# Draw the axis on the marker
frame_out = aruco.drawAxis(img, cameraMatrix, distCoeffs, rotation_vectors, translation_vectors, axisLength)




if __name__ == '__main__':
    cv2.namedWindow("Input")
    cv2.imshow("Input", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("End of file")