#!/usr/bin/env python3

import rospy
from ros_msgs.srv import LedPanel

class battery_node:
    def __init__(self):
        self.battery_level = 100
        rospy.init_node("battery_node")
        rospy.loginfo("battery_node node started")
        
        rospy.loginfo("Waiting for /set_led service")
        rospy.wait_for_service("/set_led")
        rospy.loginfo("Done")

        try:
            self.client = rospy.ServiceProxy("/set_led", LedPanel)
            response = self.client(3,False)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")
        
        rospy.Timer(rospy.Duration(7), self.set_led, oneshot=True)
    
    def set_led(self,event):
        try:
            if self.battery_level ==100:
                self.battery_level = 0
                response = self.client(3,True)
                rospy.loginfo("battery full")
                rospy.Timer(rospy.Duration(7), self.set_led, oneshot=True)
            else:
                self.battery_level = 100
                response = self.client(3,False)
                rospy.loginfo("battery empty")
                rospy.Timer(rospy.Duration(5), self.set_led, oneshot=True)
        except:
            rospy.logwarn("Service call failed")
    
def main():
    node = battery_node()
    rospy.spin()
    
if __name__ == "__main__":
    main()    
        
    