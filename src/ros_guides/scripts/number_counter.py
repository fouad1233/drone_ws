#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool,SetBoolResponse

class Counter:
    def __init__(self):
        self.counter = 0
        rospy.init_node("number_counter")
        self.sub = rospy.Subscriber("/number", Int64, self.callback)
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.srv = rospy.Service("/reset_counter", SetBool, self.handle_reset_counter)
        
    def callback(self, msg):
        count_msg = Int64()
        count_msg.data = self.counter
        self.pub.publish(count_msg)
        rospy.loginfo(f"Received: {msg.data} counter: {self.counter}")
        self.counter+=1
        
    def handle_reset_counter(self, req):
        if req.data:
            self.counter = 0
            rospy.loginfo("Counter resetted")
            return SetBoolResponse(success=True, message="Counter resetted")
        else:
            rospy.loginfo("Counter was not resetted")
            return SetBoolResponse(success=True, message="Counter was not resetted")

            
def main():
    node = Counter()
    rospy.loginfo("Number counter node started")
    rospy.spin()

if __name__ == '__main__':
    main()