#!/usr/bin/env python3
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ReceiveImage:
    
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/roscam/cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    #if q pressed, exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      sys.exit(0)
class ArucoDetection():
    def __init__(self, image, markerSize=6, totalMarkers=250,draw=True):
        self.image = image
        key = getattr(cv2.aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}' )
        self.arucoDict = cv2.aruco.Dictionary_get(key)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
    def detect_aruco(self):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.image, self.arucoDict, parameters=self.arucoParams)
        print(corners, ids, rejected)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                cv2.line(self.image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(self.image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(self.image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(self.image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(self.image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(self.image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0 ,255, 0), 2)
            return self.image,cX,cY
        return self.image,None,None
    