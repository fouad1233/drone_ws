#!/usr/bin/env python3

import cv2
import numpy as np
import cv2.aruco as aruco


# Load the predefined dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) #size is 4 by 4 and number is 50 - faster


def aruco_create():
   for i in range(6):
       # Generate the markers
       outputMarker = np.zeros((200, 200), dtype=np.uint8)
       markerImage = aruco.generateImageMarker(dictionary, i, 170, outputMarker, 1) #170 is the number of pixels
       image_path = r'/home/ume/drone_ws/src/aruco/gazebo_models-master/ar_tags/images/Marker%i.png' %i
       cv2.imwrite(image_path , markerImage)


aruco_create()
