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
    def __init__(self):
        self.x = 0
        self.y = 0
        
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

class RosNode:
    def __init__(self):
        rospy.init_node('flight_state_machine')
        #ROS Topics
        self.pub = rospy.Publisher('/flight_state', String, queue_size=10)
        self.srv = rospy.Service("/startFlight", Trigger, self.server_callback)
        self.aruco_status_sub = rospy.Subscriber("/ArucoStatus",ArucoStatus,self.aruco_status_callback)
        self.sm = smach.StateMachine(outcomes=['aborted'])
        self.sm.userdata.is_running = False

        self.control_effort_axis = Axis()
        self.ArucoStatus = ArucoStatusInstance()
        
        """
        PID Topics
        plant_node: Subscribes to control_effort and publishes on state and setpoint.
        
        
        self.pid_enable_x =         rospy.Publisher("/x_axis/pid_enable",Bool, queue_size=1)
        self.state_pub_x =          rospy.Publisher("/x_axis/state",Float64, queue_size=1)
        self.setpoint_pub_x =       rospy.Publisher("/x_axis/setpoint",Float64, queue_size=1)
        self.controleffort_sub_x =  rospy.Subscriber("/x_axis/control_effort",Float64,self.controleffort_callback_x)
        
        self.pid_enable_y =         rospy.Publisher("/y_axis/pid_enable",Bool, queue_size=1)
        self.state_pub_y =          rospy.Publisher("/y_axis/state",Float64, queue_size=1)
        self.setpoint_pub_y =       rospy.Publisher("/y_axis/setpoint",Float64, queue_size=1)
        self.controleffort_sub_y =  rospy.Subscriber("/y_axis/control_effort",Float64,self.controleffort_callback_y)
        
        #Flag to check if new control effort has been received
        self.new_control_effort_x = False 
        self.new_control_effort_y = False
        
        self.pid_enable_x.publish(False)
        self.pid_enable_y.publish(False)
        """
        
    def server_callback(self, req):
        self.sm.userdata.is_running = not self.sm.userdata.is_running
        rospy.loginfo("Main state machine is running: " + str(self.sm.userdata.is_running))
        return True, "Success"
    
    def aruco_status_callback(self,data:ArucoStatus):
        if data.found_aruco:
            self.ArucoStatus.x_state = data.x_state
            self.ArucoStatus.x_setpoint = data.x_setpoint
            self.ArucoStatus.y_state = data.y_state
            self.ArucoStatus.y_setpoint = data.y_setpoint
            self.ArucoStatus.id = data.id
            
            if not self.ArucoStatus.found_aruco:
                vehicle.set_running_flag(False)
                rospy.loginfo("Aruco found!")
                
            rospy.loginfo("Displacement Axis X: %f, Axis Y: %f", self.ArucoStatus.x_state - self.ArucoStatus.x_setpoint, 
                                                                 self.ArucoStatus.y_state - self.ArucoStatus.y_setpoint)
            rospy.loginfo("x_state: %f, x_setpoint: %f, y_state: %f, y_setpoint: %f", self.ArucoStatus.x_state, self.ArucoStatus.x_setpoint,
                                                                                      self.ArucoStatus.y_state, self.ArucoStatus.y_setpoint)
        else:
            if self.ArucoStatus.found_aruco:
              self.ArucoStatus.reset()
              rospy.loginfo("Aruco not found")
        self.ArucoStatus.found_aruco = data.found_aruco
        
    def controleffort_callback_x(self,data:Float64):
        self.control_effort_axis.x = data.data
        self.new_control_effort_x = True
        #rospy.loginfo("Control effort Axis X: %f", self.control_effort_axis.x)
        
    def controleffort_callback_y(self,data:Float64):
        self.control_effort_axis.y = data.data
        self.new_control_effort_y = True
        #rospy.loginfo("Control effort Axis Y: %f", self.control_effort_axis.y)

    
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
        smach.State.__init__(self, outcomes=['arucoLand','rtl'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
    """
    def execute(self,userdata):
        #rospy.loginfo('Executing state SEARCH')
        rate = rospy.Rate(10)
        search_thread = threading.Thread(target=vehicle.scan_rectangle_m,args=(10,10))
        vehicle.set_thread_flag(True)
        search_thread.start()
        while True:
            if node.displacement.found_aruco:
                vehicle.set_thread_flag(False)
                vehicle.stop_vehicle()
                search_thread.join()
                return 'arucoLand'
            rate.sleep()
    """
        
    def execute(self,userdata):
        #rospy.loginfo('Executing state SEARCH')
        rate = rospy.Rate(10)
        vehicle.set_running_flag(True)
        vehicle.scan_rectangle_m_vel(20,8,vel = 0.5) #Scan a 12x8m rectangle 
        vehicle.stop_vehicle()

        if node.ArucoStatus.found_aruco:
            return 'pending'
        else:
            return 'rtl'
    
class ARUCOLAND(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rtl','search','pending'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        
    def execute(self,userdata):
        rate = rospy.Rate(10)

        land_altitude = rospy.get_param('/land_altitude',0.9)
        land_velocity = rospy.get_param('/land_velocity',0.1)
        
        land_velocity = 0
        p = 1
        
        """
        node.pid_enable_x.publish(True)
        node.pid_enable_y.publish(True)
        """
        while vehicle.local_position.pose.position.z > land_altitude:
            """
            node.state_pub_x.publish(node.ArucoStatus.x_state)
            node.state_pub_y.publish(node.ArucoStatus.y_state)
            
            node.setpoint_pub_x.publish(node.ArucoStatus.x_setpoint)
            node.setpoint_pub_y.publish(node.ArucoStatus.y_setpoint)
            """
            """
            while not node.new_control_effort_x and not node.new_control_effort_y:
                rate.sleep()
            node.new_control_effort_x, node.new_control_effort_y = False, False
            """
            
            node.control_effort_axis.x = p * (node.ArucoStatus.x_state - node.ArucoStatus.x_setpoint)
            node.control_effort_axis.y = p * (node.ArucoStatus.y_state - node.ArucoStatus.y_setpoint)

            vehicle.set_velocity_once(node.control_effort_axis.x, node.control_effort_axis.y, -land_velocity, yaw_rate=0, duration=0)
            rate.sleep()
        
        vehicle.land() #If current altitude is less than land altitude, set mode to land
        """
        node.pid_enable_x.publish(False)
        node.pid_enable_y.publish(False)
        """
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