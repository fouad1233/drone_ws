#!/usr/bin/env python3
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8
from ros_msgs.msg import ArucoStatus
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import math

class ReceiveImage:
    def __init__(self):
        rospy.init_node('aruco_detect', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/roscam/cam/image_raw",Image,self.callback)
        self.aruco_detection = ArucoDetection()
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.aruco_image = self.aruco_detection.detect_aruco(cv_image)[0]
        cv2.imshow("Image window", self.aruco_image)
        #if q pressed, exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit(0)
class ArucoDetection():
    def __init__(self, markerSize=4, totalMarkers=50,draw=True):
        self.image = None
        self.position_sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.position_callback)
        key = getattr(cv2.aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}' )
        self.arucoDict = cv2.aruco.getPredefinedDictionary(key)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.fov_x = 62.2
        self.fov_y = 48.8
        self.aruco_status_pub = rospy.Publisher("/ArucoStatus",ArucoStatus,queue_size=10)
        self.ArucoStatus = ArucoStatus()
        
    def position_callback(self,data:PoseStamped):
        self.height = data.pose.position.z

    def pixels_to_meters(self,pixels,fov,res,alt):
        return pixels*( ( alt * math.tan(math.radians(fov/2)) ) / (res/2) )
    
    """
    def pixels_to_meters(self,pixels,fov,res,alt):
        return alt * math.tan(math.radians(fov/2)) * (2 * pixels / res)
        #x_distance = (altitude * tan(FOV_horizontal / 2)) * (2 * delta_x / image_width)
    """
    def detect_aruco(self,image):
        self.image = image
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.image, self.arucoDict, parameters=self.arucoParams)
        #print(ids)
                
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
                #find the center of the camera and find the distance to the center of marker, return the distance and plot a line between the two
                image_height,image_width,_ = self.image.shape
                center_point = (image_width/2,image_height/2)
                distance_to_center = cX-center_point[0] , cY-center_point[1]
             
                cv2.line(self.image,(int(center_point[0]),int(center_point[1])),(cX,cY),(0,0,255),2)
                distance_to_center_meters = (self.pixels_to_meters(distance_to_center[0],self.fov_x,image_width,self.height), self.pixels_to_meters(distance_to_center[1],self.fov_y,image_height,self.height))
                           
                           
            self.ArucoStatus.x_state = self.pixels_to_meters(cX,self.fov_x,image_width,self.height)
            self.ArucoStatus.x_setpoint = self.pixels_to_meters(center_point[0],self.fov_x,image_width,self.height)

            self.ArucoStatus.y_state = self.pixels_to_meters(cY,self.fov_y,image_width,self.height)
            self.ArucoStatus.y_setpoint = self.pixels_to_meters(center_point[1],self.fov_y,image_width,self.height)

            self.ArucoStatus.id = markerID
            self.ArucoStatus.found_aruco = True
            self.aruco_status_pub.publish(self.ArucoStatus)
            
            
            return self.image,cX,cY
            
        else:
            self.ArucoStatus.found_aruco = False #No aruco found
            self.aruco_status_pub.publish(self.ArucoStatus)   
            return self.image,None,None
            
        

if __name__ == '__main__':
    
    receive_image = ReceiveImage()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()