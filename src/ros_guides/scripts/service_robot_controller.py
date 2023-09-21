#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class RobotController:
    def __init__(self):
        rospy.init_node('turtle_controller')
        self.pub = rospy.Publisher('/turtle1/cmd_vel',  Twist, queue_size=10) 
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, callback=self.Callback)
        self.right_side = True
        
    def Callback(self,pose: Pose):
        cmd = Twist()
        cmd.linear.x,cmd.angular.z = pose.linear_velocity, pose.angular_velocity
        #rospy.loginfo(f"{cmd.linear.x}, {cmd.angular.z}")
        if pose.x > 9.0 or pose.x < 1.0 or pose.y > 9.0 or pose.y < 1.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.4
        else:   
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)
        
        if pose.x >= 5.5:
            if not self.right_side:
                self.call_set_pen_service(255,0,0,5,0)
                self.right_side = True
                rospy.loginfo("right side")

        else:
            if self.right_side:
                self.call_set_pen_service(0,255,0,5,0)
                self.right_side = False
                rospy.loginfo("left side")


    def call_set_pen_service(self,r,g,b,width,off):
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            response = set_pen(r,g,b,width,off)
            #rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")
        
        
def main():
    node = RobotController()
    rospy.wait_for_service("/turtle1/set_pen")
    node.call_set_pen_service(255,0,0,5,0)  
    rospy.loginfo("robot_controller node started") 
    rospy.spin()

if __name__ == "__main__":
    main()