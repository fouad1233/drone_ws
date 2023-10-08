#!/usr/bin/env python3

import rospy
import smach
import time
import threading
from std_msgs.msg import String
from std_srvs.srv import Trigger
from movement import Control

class RosNode:
    def __init__(self):
        rospy.init_node('flight_state_machine')
        self.pub = rospy.Publisher('/flight_state', String, queue_size=10)
        self.srv = rospy.Service("/startFlight", Trigger, self.server_callback)
        self.sm = smach.StateMachine(outcomes=['aborted'])
        self.sm.userdata.is_running = False
        
    def server_callback(self, req):
        self.sm.userdata.is_running = not self.sm.userdata.is_running
        rospy.loginfo("Main state machine is running: " + str(self.sm.userdata.is_running))
        return True, "Success"
    
    def get_sm(self):
        return self.sm

class PENDING(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['aborted','pending','start'],
                             input_keys=['isRunning'])
        
    def execute(self,userdata):
        rospy.loginfo('PENDING')
        if rospy.is_shutdown():
            return 'aborted'
        elif userdata.isRunning:
            return 'start'
        else:
            return 'pending'
        
class ARM(smach.State):
    def __init__(self,control:Control):
        smach.State.__init__(self, outcomes=['armed','notArmable','failed'])
        self.control = control

    def execute(self,userdata):
        rospy.loginfo('Executing state ARM')
        
        if not self.control.drone.is_armable:
            return 'notArmable'
        elif self.control.arm():
            return 'armed'
        else:
            return 'failed'

class TAKEOFF(smach.State):
    def __init__(self,control:Control):
        smach.State.__init__(self, outcomes=['tookOff'])
        self.control = control
        self.altitude = rospy.get_param('takeoff_altitude',3)
        
    def execute(self,userdata):
        rospy.loginfo('Executing state TAKEOFF')
        self.control.takeoff(self.altitude)
        return 'tookOff'

class SEARCH(smach.State):
    def __init__(self,control:Control):
        smach.State.__init__(self, outcomes=['arucoLand'])
        self.control = control
        
    def execute(self,userdata):
        rospy.loginfo('Executing state SEARCH')
        self.control.guided()
        self.control.scan_rectangle_m(10,10)
        return 'arucoLand'
    
class ARUCOLAND(smach.State):
    def __init__(self,control:Control):
        smach.State.__init__(self, outcomes=['rtl','search'])
        self.control = control
        
    def execute(self,userdata):
        rospy.loginfo('Executing state ARUCOLAND')
        return 'rtl'

class RTL(smach.State):
    def __init__(self,control:Control):
        smach.State.__init__(self, outcomes=['pending','aborted'],
                             input_keys=['isRunning'],
                             output_keys=['isRunning'])
        self.control = control
        
    def execute(self,userdata):
        rospy.loginfo('Executing state RTL')
        self.control.rtl()
        time.sleep(10)
        isRunning = userdata.isRunning
        userdata.isRunning = False
        if isRunning:
            return 'pending'
        else:
            return 'aborted'
    
def main():
    node = RosNode()
    sm = node.get_sm()
    control = Control()
    with sm:
        # Add states to the container
        smach.StateMachine.add('PENDING', PENDING(), 
                               transitions={'aborted':'aborted',
                                            'pending':'PENDING', 
                                            'start':'ARM'},
                               remapping={'isRunning':'is_running'})
        smach.StateMachine.add('ARM', ARM(control), 
                               transitions={'armed':'TAKEOFF', 
                                            'notArmable':'PENDING',
                                            'failed':'PENDING'})
        smach.StateMachine.add('TAKEOFF', TAKEOFF(control), 
                               transitions={'tookOff':'SEARCH'})
        smach.StateMachine.add('SEARCH', SEARCH(control), 
                               transitions={'arucoLand':'ARUCOLAND'})
        smach.StateMachine.add('ARUCOLAND', ARUCOLAND(control),
                                 transitions={'search':'SEARCH',
                                              'rtl':'RTL'})
        smach.StateMachine.add('RTL', RTL(control),
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