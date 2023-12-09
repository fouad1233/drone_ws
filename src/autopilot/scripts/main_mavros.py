#!/usr/bin/env python3

import rospy
import smach
import time
import threading
from std_msgs.msg import String, Int8, Float64, Bool
from std_srvs.srv import Trigger
from vehicle import Vehicle

class RosNode:
    def __init__(self):
        rospy.init_node('flight_state_machine')
        #ROS Topics
        self.pub = rospy.Publisher('/flight_state', String, queue_size=10)
        self.srv = rospy.Service("/startFlight", Trigger, self.server_callback)
        self.aruco_find_sub = rospy.Subscriber("/aruco_find",Int8,self.aruco_find_callback)
        self.sm = smach.StateMachine(outcomes=['aborted'])
        self.sm.userdata.is_running = False
        self.aruco_found = False
        
        #PID Topics
        self.setpoint_pub = rospy.Publisher("/setpoint",Float64, queue_size=1)
        self.state_pub = rospy.Publisher("/state",Float64, queue_size=1)
        self.controleffort_sub = rospy.Subscriber("/control_effort",Float64,self.controleffort_callback)
        self.enable_pid_pub = rospy.Publisher("/pid_enable",Bool, queue_size=1)
        
        self.enable_pid_pub.publish(False)
        
    def server_callback(self, req):
        self.sm.userdata.is_running = not self.sm.userdata.is_running
        rospy.loginfo("Main state machine is running: " + str(self.sm.userdata.is_running))
        return True, "Success"
    
    def aruco_find_callback(self,data:Int8):
        if data.data == -1:
            rospy.loginfo("Aruco not found")
            self.aruco_found = False
        else:
            rospy.loginfo("Aruco found")
            self.aruco_found = True
            
    def controleffort_callback(self,data:Float64):
        self.output = data.data
    
    def get_sm(self):
        return self.sm

class PENDING(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['aborted','pending','start'],
                             input_keys=['isRunning'])
        self.rate = rospy.Rate(1)
        
    def execute(self,userdata):
        #rospy.loginfo('PENDING')
        while True:
            if rospy.is_shutdown():
                return 'aborted'
            elif userdata.isRunning:
                return 'start'
            self.rate.sleep()
        
class ARM(smach.State):
    def __init__(self,vehicle:Vehicle):
        smach.State.__init__(self, outcomes=['armed','notArmable','failed'])
        self.vehicle = vehicle

    def execute(self,userdata):
        #rospy.loginfo('Executing state ARM')
        if self.vehicle.guided():
            rospy.loginfo('guided')
        else:
            return 'failed'
        if self.vehicle.arm():
            return 'armed'
        else:
            return 'failed'

class TAKEOFF(smach.State):
    def __init__(self,vehicle:Vehicle):
        smach.State.__init__(self, outcomes=['tookOff'])
        self.vehicle = vehicle
        self.altitude = rospy.get_param('takeoff_altitude',3)
        
    def execute(self,userdata):
        #rospy.loginfo('Executing state TAKEOFF')
        self.vehicle.takeoff(self.altitude,10)
        return 'tookOff'

class SEARCH(smach.State):
    def __init__(self,vehicle:Vehicle,node:RosNode):
        smach.State.__init__(self, outcomes=['arucoLand'])
        self.vehicle = vehicle
        self.rosnode = node
        
    def execute(self,userdata):
        #rospy.loginfo('Executing state SEARCH')
        rate = rospy.Rate(10)
        search_thread = threading.Thread(target=self.vehicle.scan_rectangle_m,args=(10,10))
        self.vehicle.set_thread_flag(True)
        search_thread.start()
        while True:
            if self.rosnode.aruco_found:
                self.vehicle.set_thread_flag(False)
                search_thread.join()
                return 'arucoLand'
            rate.sleep()
    
class ARUCOLAND(smach.State):
    def __init__(self,vehicle:Vehicle,node:RosNode):
        smach.State.__init__(self, outcomes=['rtl','search'])
        self.vehicle = vehicle
        self.rosnode = node
        
    def execute(self,userdata):
        rate = rospy.Rate(10)
        self.vehicle.stop_vehicle()
        while True:
            rate.sleep()
        
        return 'rtl'

class RTL(smach.State):
    def __init__(self,vehicle:Vehicle):
        smach.State.__init__(self, outcomes=['pending','aborted'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        self.vehicle = vehicle
        
    def execute(self,userdata):
        #rospy.loginfo('Executing state RTL')
        self.vehicle.rtl()
        rospy.Rate(1/10).sleep()
        isRunning = userdata.isRunning
        userdata.isRunning = False
        if isRunning:
            return 'pending'
        else:
            return 'aborted'
    
def main():
    node = RosNode()
    vehicle = Vehicle()
    sm = node.get_sm()
    with sm:
        # Add states to the container
        smach.StateMachine.add('PENDING', PENDING(), 
                               transitions={'aborted':'aborted',
                                            'pending':'PENDING', 
                                            'start':'ARM'},
                               remapping={'isRunning':'is_running'})
        smach.StateMachine.add('ARM', ARM(vehicle), 
                               transitions={'armed':'TAKEOFF', 
                                            'notArmable':'PENDING',
                                            'failed':'PENDING'})
        smach.StateMachine.add('TAKEOFF', TAKEOFF(vehicle), 
                               transitions={'tookOff':'SEARCH'})
        smach.StateMachine.add('SEARCH', SEARCH(vehicle,node), 
                               transitions={'arucoLand':'ARUCOLAND'})
        smach.StateMachine.add('ARUCOLAND', ARUCOLAND(vehicle,node),
                                 transitions={'search':'SEARCH',
                                              'rtl':'RTL'})
        smach.StateMachine.add('RTL', RTL(vehicle),
                                    transitions={'pending':'PENDING',
                                                 'aborted': 'aborted'},
                                    remapping={'isRunning':'is_running'})

    # Execute SMACH plan
    #outcome = sm.execute()
    
    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()
    

    # Block until everything is preempted 
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()
    

if __name__ == "__main__":
    main()