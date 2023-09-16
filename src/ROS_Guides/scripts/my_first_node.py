#!/usr/bin/env python3
import rospy # Import the Python library for ROS





if __name__ == "__main__":
    rospy.init_node('test_node') # Initiate a Node named 'my_first_python_node'
    rospy.loginfo("This node has been started") # Print info message to the console
    rospy.logdebug("This is a debug message.") # Print debug message to the console
    rospy.logwarn("This is a warning message.") # Print warning message to the console
    rospy.logerr("This is an error message.") # Print error message to the console
    rospy.logfatal("This is a fatal error message.") # Print fatal error message to the console
    
    rate = rospy.Rate(10) # Set a publish rate of 10 Hz
    while not rospy.is_shutdown(): # Create a loop that will go until someone stops the program execution
        rospy.loginfo("Hello World") # Print "Hello World" to the console
        rate.sleep() # Make sure the publish rate maintains at 10 Hz
