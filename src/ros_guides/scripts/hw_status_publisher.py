#!/usr/bin/env python3

import rospy
from ros_msgs.msg import HardwareStatus

class HWStatusPublisher:
    def __init__(self):
        rospy.init_node("hw_status_publisher")
        self.pub = rospy.Publisher("/my_robot/hw_status", HardwareStatus, queue_size=10)
        rospy.Timer(rospy.Duration(1), self.publish_status)

    def publish_status(self, event:rospy.timer.TimerEvent=None):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_up = True
        msg.debug_message = "Everything is OK"
        self.pub.publish(msg)
        rospy.loginfo("Publishing hardware status")

def main():
    node = HWStatusPublisher()
    rospy.loginfo("HW Status publisher node started")
    rospy.spin()

if __name__ == '__main__':
    main()