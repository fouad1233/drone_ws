#!/usr/bin/env python3

import rospy
from rospy_tutorials.srv import AddTwoInts

class AddTwoIntsClient:
    def __init__(self):
        rospy.init_node("add_two_ints_client")
        rospy.wait_for_service("/add_two_ints")
        
        try:
            self.client = rospy.ServiceProxy("/add_two_ints", AddTwoInts)
            response = self.client(5,6)
            rospy.loginfo(f"the sum is {response.sum}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")
        
        

def main():
    node = AddTwoIntsClient()
    rospy.loginfo("add_two_ints_client node started")
    rospy.spin()

if __name__ == '__main__':
    main()