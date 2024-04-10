#!/usr/bin/env python3

import rospy
import smach
import time
import threading
from std_msgs.msg import String, Float64, Bool
from std_srvs.srv import Trigger
from ros_msgs.msg import ArucoStatus
from vehicle import Vehicle

class Axis:
    def __init__(self,x=0,y=0):
        self.x = 0
        self.y = 0
        
    def __str__(self):
        return "X: " + str(self.x) + "Y: " + str(self.y)
        
class ArucoStatusInstance:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x_state = 0
        self.x_setpoint = 0
        self.y_state = 0
        self.y_setpoint = 0
        self.id = 0
        self.found_aruco = False
        self.aruco_not_found_counter = 0
        
class ArucoMarker(Axis):
    def __init__(self, id, x, y):
        super().__init__(x,y)
        self.id = id
        
    def __str__(self):
        return "ID: " + str(self.id) + " Coordinates: " + str((self.x,self.y))


class RosNode:
    def __init__(self):
        rospy.init_node('flight_state_machine')
        #ROS Topics
        self.pub = rospy.Publisher('/flight_state', String, queue_size=10)
        self.srv = rospy.Service("/startFlight", Trigger, self.server_callback)
        self.aruco_status_sub = rospy.Subscriber("/ArucoStatus",ArucoStatus,self.aruco_status_callback)
        self.sm = smach.StateMachine(outcomes=['aborted'])
        self.sm.userdata.is_running = False

        self.ArucoStatus = ArucoStatusInstance()
        
        self.ArucoMarkers = [ArucoMarker(3, 10 ,10),
                             ArucoMarker(4, 10 ,0),
                             ArucoMarker(5, 10 ,-10),
                             ArucoMarker(2, -10,-10),
                             ArucoMarker(1, -10,0),
                             ArucoMarker(0, -10,10)]

    def server_callback(self, req):
        self.sm.userdata.is_running = not self.sm.userdata.is_running
        rospy.loginfo("Main state machine is running: " + str(self.sm.userdata.is_running))
        return True, "Success"
    
    def aruco_status_callback(self,data:ArucoStatus):
        if data.found_aruco:
            self.copy_aruco_status(self.ArucoStatus,data)
            self.ArucoStatus.aruco_not_found_counter = 0
            
            if not self.ArucoStatus.found_aruco:
                vehicle.set_running_flag(False)
                rospy.loginfo("Aruco found!")

        else:
            self.ArucoStatus.aruco_not_found_counter += 1
            if self.ArucoStatus.aruco_not_found_counter > 5: #If Aruco not found for 10 consecutive frames
                self.ArucoStatus.reset()
            
            if self.ArucoStatus.found_aruco:
                rospy.loginfo("Aruco not found")
        self.ArucoStatus.found_aruco = data.found_aruco
        
    def copy_aruco_status(self, arucoobj:ArucoStatus, data:ArucoStatus):
        arucoobj.x_state = data.x_state
        arucoobj.x_setpoint = data.x_setpoint
        arucoobj.y_state = data.y_state
        arucoobj.y_setpoint = data.y_setpoint
        arucoobj.id = data.id
    
    def get_sm(self):
        return self.sm
    
node = RosNode()
vehicle = Vehicle()

class PENDING(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['pending','start'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        self.rate = rospy.Rate(1)
        
    def execute(self,userdata):
        #rospy.loginfo('PENDING')

        while True:
            if userdata.isRunning:
                return 'start'
            self.rate.sleep()
        
class ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['armed','notArmable','failed'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])

    def execute(self,userdata):
        #rospy.loginfo('Executing state ARM')
        if vehicle.guided():
            rospy.loginfo('guided')
        else:
            rospy.loginfo('Could not set vehicle to guided mode')
            userdata.isRunning = False
            return 'failed'
        if vehicle.arm():
            return 'armed'
        else:
            rospy.loginfo('Could not arm vehicle')
            userdata.isRunning = False
            return 'failed'

class TAKEOFF(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tookOff','failed'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])

    def execute(self,userdata):
        #rospy.loginfo('Executing state TAKEOFF')
        altitude = rospy.get_param('takeoff_altitude',3)
        if vehicle.takeoff(altitude,timeout = 10):
            return 'tookOff'
        else:
            vehicle.disarm()
            userdata.isRunning = False
            return 'failed'
        
class SEARCH(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arucoLand','pending','rtl'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        
    def execute(self,userdata):
        #rospy.loginfo('Executing state SEARCH')
        rate = rospy.Rate(10)
        vehicle.set_running_flag(True)
        vehicle.scan_rectangle_m(12,8) #Scan a 12x8m rectangle 
        vehicle.stop_vehicle()

        if node.ArucoStatus.found_aruco:
            return 'arucoLand'
        else:
            return 'rtl'
    
class ARUCOLAND(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rtl','search','pending'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        
    def execute(self,userdata):
        rate = rospy.Rate(10)

        land_altitude = rospy.get_param('/land_altitude',0.6)
        land_velocity = rospy.get_param('/land_velocity',0.4)
        
        p = 2
        
        control_effort_axis = Axis()

        while vehicle.local_position.pose.position.z > land_altitude:
            
            control_effort_axis.x = p * (node.ArucoStatus.x_setpoint - node.ArucoStatus.x_state)
            control_effort_axis.y = p * (node.ArucoStatus.y_setpoint - node.ArucoStatus.y_state)

            rospy.loginfo("Displacement Axis X: %f, Axis Y: %f", control_effort_axis.x/p, control_effort_axis.y/p)
            rospy.loginfo("Control Effort X: %f, Control Effort Y: %f", control_effort_axis.x, control_effort_axis.y)

            vehicle.send_ned_velocity(control_effort_axis.y, control_effort_axis.x, -land_velocity, yaw_rate=0, duration=0)
            rate.sleep()
        
        node.ArucoStatus.reset()
        vehicle.land() #If current altitude is less than land altitude, set mode to land

        userdata.isRunning = False
        return 'pending'

class RTL(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pending','aborted'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        
    def execute(self,userdata):
        #rospy.loginfo('Executing state RTL')
        vehicle.rtl()
        rospy.Rate(1/10).sleep()
        isRunning = userdata.isRunning
        userdata.isRunning = False
        if isRunning:
            return 'pending'
        else:
            return 'aborted'
    
def main():
    sm = node.get_sm()

    with sm:
        # Add states to the container
        smach.StateMachine.add('PENDING', PENDING(), 
                               transitions={'pending':'PENDING', 
                                            'start':'ARM'},
                               remapping={'isRunning':'is_running'})
        smach.StateMachine.add('ARM', ARM(), 
                               transitions={'armed':'TAKEOFF', 
                                            'notArmable':'PENDING',
                                            'failed':'PENDING'},
                               remapping={'isRunning':'is_running'})
        smach.StateMachine.add('TAKEOFF', TAKEOFF(), 
                               transitions={'tookOff':'SEARCH',
                                            'failed':'PENDING'},
                               remapping={'isRunning':'is_running'})
        smach.StateMachine.add('SEARCH', SEARCH(), 
                               transitions={'arucoLand':'ARUCOLAND',
                                            'pending':'PENDING',
                                            'rtl':'RTL'},
                               remapping={'isRunning':'is_running'})
        smach.StateMachine.add('ARUCOLAND', ARUCOLAND(),
                                 transitions={'search':'SEARCH',
                                              'rtl':'RTL',
                                              'pending':'PENDING'},
                                 remapping={'isRunning':'is_running'})
        smach.StateMachine.add('RTL', RTL(),
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