#!/usr/bin/env python3
import rospy
from ros_msgs.srv import LedPanel, LedPanelResponse

class ledPanel:
    def __init__(self):
        self.led_stats = [False, False, False]
        rospy.init_node("led_panel_node")
        rospy.loginfo("led_panel_node node started")
        self.srv = rospy.Service("/set_led", LedPanel, self.set_led)
        rospy.loginfo("Service /set_led is ready")
        
    def set_led(self, req):
        if req.led_number<=3:
            self.led_stats[req.led_number-1] = req.state
            rospy.loginfo(f"Led {req.led_number} is set to {req.state}")
            return True
        else:
            rospy.logerr(f"led_{req.led_number} was not found!")
            return False
    
    
def main():
    node = ledPanel()
    rospy.spin()
    
if __name__ == "__main__":
    main()