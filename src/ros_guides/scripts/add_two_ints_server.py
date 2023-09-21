#!/usr/bin/env python3

import rospy
from rospy_tutorials.srv import AddTwoInts

class AddTwoIntsServer:
    def __init__(self):
        rospy.init_node('add_two_ints_server')
        self.srv = rospy.Service('/add_two_ints', AddTwoInts, self.handle_add_two_ints)
        rospy.loginfo("Service server has been started")
        
    def handle_add_two_ints(self, req):
        sum = req.a + req.b
        rospy.loginfo(f"sum is {sum}")
        return sum



def main():
    rospy.loginfo("add_two_ints_server node started")
    node = AddTwoIntsServer()   
    rospy.spin()


if __name__ == '__main__':
    main()