#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

class Publisher:
    def __init__(self):
        rospy.init_node('number_publisher')
        
        self.publishing_freq = rospy.get_param('/publishing_freq', default = 1)
        self.num = rospy.get_param('/number_to_publish', default = 42)
        rospy.set_param("/another_param","hello")
        
        self.pub = rospy.Publisher('/number', Int64, queue_size=10)
        rospy.Timer(rospy.Duration(1/self.publishing_freq), self.timer_callback)
        
    def timer_callback(self, event:rospy.timer.TimerEvent=None):
        msg = Int64()
        msg.data = self.num
        self.pub.publish(msg)        
        rospy.loginfo(f"Number published {msg.data}")  
        
def main():
    node = Publisher()
    rospy.loginfo("Number publisher node started")
    rospy.spin()
    

if __name__ == '__main__':
    main()