#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('draw_circle') # Initiate a Node named 'draw_circle'
    rospy.loginfo("draw_circle node has been started") # Print info message to the console
    
    pub = rospy.Publisher('/turtle1/cmd_vel',  Twist, queue_size=10) 
    rate = rospy.Rate(2) # Set a publish rate of 2 Hz

    while not rospy.is_shutdown(): # Create a loop that will go until someone stops the program execution
        twist_msg = Twist()
        twist_msg.linear.x = 2
        twist_msg.angular.z = 1
        pub.publish(twist_msg)
        rate.sleep() 
        
if __name__ == "__main__":
    main()
    