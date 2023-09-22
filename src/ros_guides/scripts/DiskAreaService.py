#!/usr/bin/env python3

import rospy
from ros_msgs.srv import ComputeDiskArea,ComputeDiskAreaResponse
from math import pi

class DiskAreaService:
    def __init__(self):
        rospy.init_node("disk_area_service")
        self.srv = rospy.Service("/compute_disk_area", ComputeDiskArea, self.handle_compute_disk_area_msg)
        rospy.loginfo("Service server has been started")

        
    def handle_compute_disk_area(self, req):
        area = pi*(req.radius)**2
        rospy.loginfo(f"Area is: {area}")
        return area
    
    def handle_compute_disk_area_msg(self, req):
        area = pi*(req.radius)**2
        rospy.loginfo(f"Area is: {area}")
        return ComputeDiskAreaResponse(area=area)
    
    
def main():
    rospy.loginfo("add_two_ints_server node started")
    node = DiskAreaService()
    rospy.spin()
    
    
if __name__ == '__main__':
    main()