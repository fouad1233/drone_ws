#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

class Publisher:
    def __init__(self):
        rospy.init_node('number_publisher')
        self.pub = rospy.Publisher('number', Int64, queue_size=10)
        rospy.Timer(rospy.Duration(1), self.timer_callback)
        
    def timer_callback(self, event:rospy.timer.TimerEvent=None):
        msg = Int64()
        msg.data = 42
        self.pub.publish(msg)
        #rospy.loginfo(f"time difference is: {event.current_real-event.current_expected}")
        rospy.loginfo("pubslishing 42")
        
        
def main():
    node = Publisher()
    rospy.loginfo("Number publisher node started")
    rospy.spin()
    

if __name__ == '__main__':
    main()