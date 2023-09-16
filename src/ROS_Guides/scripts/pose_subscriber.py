#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def poseCallback(pose_message: Pose):
    rospy.loginfo(f"({pose_message.x}, {pose_message.y}, {pose_message.theta})")
    
    
def main():
    rospy.init_node('turtle_pose_subscriber', anonymous=True) # Initiate a Node named 'pose_subscriber'
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback=poseCallback) # Create a Subscriber object that will listen to the /turtle1/pose topic and will call the poseCallback() function each time it reads something from the topic
    rate = rospy.Rate(2) # Set a publish rate of 2 Hz
    
    rospy.spin() # Create a loop that will keep the program in execution
    
    
if __name__ == "__main__":
    main()