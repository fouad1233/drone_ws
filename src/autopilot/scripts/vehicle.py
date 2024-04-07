#! /usr/bin/env python3

import rospy
from pymavlink import mavutil
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped

from mavros_msgs.msg import HomePosition, PositionTarget, State,\
                            WaypointList, ParamValue
from mavros_msgs.srv import CommandBool, CommandTOL, CommandTOLRequest, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush, ParamGetRequest, ParamSetRequest
from sensor_msgs.msg import NavSatFix, Imu

class Vehicle:
    def __init__(self):
        #rospy.init_node('vehicle', anonymous=True)
        self.running = True
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None
        
        self.x ,self.y ,self.z = 0,0,0
        
        self.sub_topics_ready = {
            key: False
            for key in [
                'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }
        
        self.action_fun = {
            "arm": self.arm,
            "rtl": self.rtl,
            "guided": self.guided,
            "land": self.land,
            "takeoff": self.takeoff,
            "scan": self.scan_rectangle_m,
            "goto": self.goto_position_target_local_ned,
            "velocity": self.send_ned_velocity
        }
        
        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/cmd/takeoff', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)

            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")
            raise Exception("failed to connect to services")
        
        """
        self.set_stream_rate_srv = rospy.ServiceProxy('mavros/set_stream_rate', 
                                                     StreamRate)
        
        local_position_id = mavutil.mavlink.MAVLink_msgid.LOCAL_POSITION_NED
        self.set_stream_rate_srv(local_position_id, 30)"""
        
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get',     
                                                ParamGet)
        self.set_param_srv = rospy.ServiceProxy('mavros/param/set', 
                                                ParamSet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', 
                                            CommandTOL)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', 
                                               SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)
        
        # ROS subscribers
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', 
                                               WaypointList, 
                                               self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', 
                                          State, 
                                          self.state_callback)
        # ROS publishers
        self.velocity_setpoint_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",
                                                            TwistStamped,
                                                            queue_size=10)
                
        self.setpoint_local_pub = rospy.Publisher('/mavros/setpoint_position/local', 
                                            PoseStamped, 
                                            queue_size=10)
        self.setpoint_global_pub = rospy.Publisher('/mavros/setpoint_position/global', 
                                            GeoPoseStamped, 
                                            queue_size=10)
        
        #Timer for publishing velocity message
        self.velocity_msg = TwistStamped()
        
    
    def set_running_flag(self,running_flag):
        self.running = running_flag
    
    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo(f"current mission waypoint sequence updated: {data.current_seq}")

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data: State):
        if self.state.armed != data.armed:
            rospy.loginfo(f"armed state changed from {self.state.armed} to {data.armed}")

        if self.state.connected != data.connected:
            rospy.loginfo(f"connected changed from {self.state.connected} to {data.connected}")

        if self.state.mode != data.mode:
            rospy.loginfo(f"mode changed from {self.state.mode} to {data.mode}")

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo(f"setting FCU arm: {arm}")
        loop_freq = 1  # Hz
        rate = rospy.Rate(1)
        for i in range(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo("Armed")
                return True
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()
        
        return False
    

    def takeoff(self, altitude, timeout):
        """Take off to a specified altitude (in meters).

        Args:
            altitude (float): The desired altitude for takeoff.
            timeout (int): The maximum time to wait for takeoff (in seconds).

        Returns:
            bool: True if the takeoff was successful, False otherwise.
        """
        rospy.loginfo(f"Initiating takeoff to {altitude} meters.")
        loop_freq = 1  # Frequency in Hz
        rate = rospy.Rate(loop_freq)
        self.z = altitude
        takeoff_cmd = CommandTOLRequest()
        takeoff_cmd.yaw = 0
        takeoff_cmd.min_pitch = 0
        takeoff_cmd.latitude = 0
        takeoff_cmd.longitude = 0
        takeoff_cmd.altitude = altitude

        # Send an arming command to prepare for takeoff
        res = self.takeoff_client(takeoff_cmd)

        for i in range(timeout * loop_freq):
            current_altitude = self.local_position.pose.position.z
            # Check if the vehicle is disarmed
            if not self.state.armed:
                rospy.logerror("Drone is not armed.")
                break

            rospy.loginfo(f"Taking off ({current_altitude}m)")

            # Check if the current altitude is close to the desired altitude
            if current_altitude >= altitude * 0.9:
                rospy.loginfo("Takeoff completed.")
                return True

            # If the previous take off command was not successful, try again
            if not res.success:
                try:
                    rospy.logerr("Failed to send takeoff command. Trying again.")
                    res = self.takeoff_client(takeoff_cmd)
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rate.sleep()

        rospy.logerr("Takeoff failed.")
        return False

    def set_mode(self, mode:str, timeout:int = 5):
        """mode: APM mode string, timeout(int): seconds"""
        rospy.loginfo(f"setting FCU mode: {mode}")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if self.state.mode == mode:
                rospy.loginfo(f"mode set to {mode}")
                return True
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()

        return False
    
    def get_param(self, param: str):
        data = ParamGetRequest()
        data.param_id = param
        result = self.get_param_srv(data)

        return result.success, result.value.integer, result.value.real

    def set_param(self, param_id, param_value: ParamValue, timeout):
        """param: APM param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        rospy.loginfo(f"setting APM parameter: {param_id} with value {value}")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            try:
                res = self.set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {param_id} set to {value}")
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)
                
            rate.sleep()

        return False
    
    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in range(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready")
                return True

            rate.sleep()

        return False
        
    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                rospy.loginfo("clear waypoints success")
                return True
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()

        return False

    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in range(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo(f"number of waypoints transferred: {len(waypoints)}")
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success")
                return True
            
            rate.sleep()
            
        return False
        
    def arm(self):
        return self.set_arm(True, timeout = 5)
    
    def disarm(self):
        return self.set_arm(False,timeout = 5)
            
    def rtl(self):
        return self.set_mode("RTL")
        
    def guided(self):
        return self.set_mode("GUIDED")
    
    def land(self):
        return self.set_mode("LAND")
    
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, yaw_rate = 0, duration = 0):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with linear
        velocities vx, vy, vz and angular velocity yaw_rate.
        """
        self.velocity_msg.twist.linear.x = velocity_x
        self.velocity_msg.twist.linear.y = velocity_y
        self.velocity_msg.twist.linear.z = velocity_z
        self.velocity_msg.twist.angular.z = yaw_rate
        
        self.velocity_setpoint_publisher.publish(self.velocity_msg)
        
    def is_reached(self, destination, current_pos, threshold=0.1):
        return abs(destination - current_pos) < threshold
        
    def goto_position_target_local_ned(self, north, east, down):
        position_target = PoseStamped()
        position_target.pose.position.x = north
        position_target.pose.position.y = east
        position_target.pose.position.z = down

        self.setpoint_local_pub.publish(position_target)

    def goto_position_target_relative_ned(self, north, east, down):
        position_target = PoseStamped()
        position_target.pose.position.x = north
        position_target.pose.position.y = east
        position_target.pose.position.z = down

        self.setpoint_local_pub.publish(position_target)
        
    def go_in_x_m(self,m):
        if not self.running:
            return
        destination = self.local_position.pose.position.x + m
        self.goto_position_target_local_ned(destination,
                                            self.local_position.pose.position.y,
                                            self.local_position.pose.position.z)
        #wait until drone reaches the target
        rate = rospy.Rate(30)
        while self.running and not self.is_reached(destination, self.local_position.pose.position.x):
            rate.sleep()
            
    def go_in_y_m(self,m):
        if not self.running:
            return
        destination = self.local_position.pose.position.y + m
        self.goto_position_target_local_ned(self.local_position.pose.position.x,
                                            destination,
                                            self.local_position.pose.position.z)
        #wait until drone reaches the target
        rate = rospy.Rate(30)
        while self.running and not self.is_reached(destination, self.local_position.pose.position.y):
            rate.sleep()
            
    def go_in_z_m(self,m):
        if not self.running:
            return
        destination = self.local_position.pose.position.z + m
        self.goto_position_target_local_ned(self.local_position.pose.position.x,
                                            self.local_position.pose.position.y,
                                            destination)
        #wait until drone reaches the target
        rate = rospy.Rate(30)
        while self.running and not self.is_reached(destination, self.local_position.pose.position.z):
            rate.sleep()
            
    def go_in_x_y_z_m(self,x_m,y_m,z_m):
        self.x = self.local_position.pose.position.x + x_m
        self.y = self.local_position.pose.position.y + y_m
        self.z = self.local_position.pose.position.z + z_m
        self.goto_position_target_local_ned(self.x,self.y,self.z)
        
    def scan_rectangle_m(self,x,y):
        self.go_in_y_m(y)
        self.go_in_x_m(x)
        self.go_in_y_m(-2*y)
        self.go_in_x_m(-2*x)
        self.go_in_y_m(2*y)
        self.go_in_x_m(x)
        self.go_in_y_m(-y)
    
    def stop_vehicle(self):
        return self.send_ned_velocity(0, 0, 0, 0, 0)
    
    def condition_yaw(self,heading, relative=False):
        # Set the desired yaw angle in radians
        setpoint_msg = PoseStamped()
        setpoint_msg.w = heading
        try:
            if relative:
                self.setpoint_local_pub.publish(setpoint_msg)
            else:
                self.setpoint_global_pub.publish(setpoint_msg)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            
    def turn_to_north(self):
        self.condition_yaw(0, False)
    
    def get_location_metres(self,original_location, dNorth, dEast):
        pass

    def get_distance_metres(self,aLocation1, aLocation2):
        pass

    def get_bearing(self,aLocation1, aLocation2):
        pass
    
    def add_mission(self):
        pass
    