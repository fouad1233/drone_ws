#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class RobotController:
    def __init__(self):
        rospy.init_node('turtle_controller')
        self.pub = rospy.Publisher('/turtle1/cmd_vel',  Twist, queue_size=10) 
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, callback=self.Callback)

    def Callback(self,pose: Pose):
        cmd = Twist()
        cmd.linear.x,cmd.angular.z = pose.linear_velocity, pose.angular_velocity
        rospy.loginfo(f"{cmd.linear.x}, {cmd.angular.z}")
        if pose.x > 9.0 or pose.x < 1.0 or pose.y > 9.0 or pose.y < 1.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.4
        else:   
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)
        
def main():
    node = RobotController()
    rospy.loginfo("robot_controller node started") 
    rospy.spin()

if __name__ == "__main__":
    main()