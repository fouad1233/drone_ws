#!/usr/bin/env python3
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import dronekit
import time, math
import argparse
from pymavlink import mavutil

class Control:
    def __init__(self, drone_url='127.0.0.1:14550', baudrate=115200):
        self.drone = connect(drone_url, wait_ready=False, baud=baudrate)
        self.command = self.drone.commands
        self.x = 0
        self.y = 0
        self.z = 0
    
    def get_drone_obj(self):
        return self.drone
    
    def arm(self):
        if not self.drone.is_armable:
            print("Drone is not armable")
            time.sleep(1)
            return False
        
        print("Drone is armable")
        self.drone.mode = VehicleMode("GUIDED")
        time.sleep(1)
        print(self.drone.mode)

        self.drone.armed = True
        while not self.drone.armed:
            print("Drone is getting armed")
            time.sleep(0.5)
        print("Drone armed")
        return True
        
    def takeoff(self, altitude):
        self.z = altitude
        self.drone.simple_takeoff(altitude)
        current_altitude = self.drone.location.global_relative_frame.alt
        while current_altitude < altitude*0.9:
            current_altitude = self.drone.location.global_relative_frame.alt
            print(f"Taking off ({current_altitude}m)")
            time.sleep(1)
            
    def rtl(self):
        self.drone.mode = VehicleMode("RTL")
        
    def guided(self):
        self.drone.mode = VehicleMode("GUIDED")
        
    def add_mission(self):
        
        self.command.clear()
        time.sleep(1)
        
        altitude = 10
        #TAKE OFF
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,0,altitude))
        #WAYPOINT
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,-35.36262207,149.16511350,20))
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,-35.36322841, 149.16584325,30))
        #RTL
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))
        #Validation
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))

        self.command.upload()
        print("Commands uploading...")

    def goto_position_target_local_ned(self,north, east, down): #relative to the home location
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.drone.send_mavlink(msg)
        
    def goto_position_target_relative_ned(self,north, east, down): #relative to the current location
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.drone.send_mavlink(msg)
        
    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


        # send command to vehicle on 1 Hz cycle
        for _ in range(0,duration):
            self.drone.send_mavlink(msg)
            time.sleep(1)


    def condition_yaw(self,heading, relative=False):
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.drone.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.drone.send_mavlink(msg)

    def turn_to_north(self):
        self.condition_yaw(0, False)
        
    """
    Functions to make it easy to convert between the different frames-of-reference. In particular these
    make it easy to navigate in terms of "metres from the current position" when using commands that take 
    absolute positions in decimal degrees.

    The methods are approximations only, and may be less accurate over longer distances, and when close 
    to the Earth's poles.

    Specifically, it provides:
    * get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
    * get_distance_metres - Get the distance between two LocationGlobal objects in metres
    * get_bearing - Get the bearing in degrees to a LocationGlobal
    """

    def get_location_metres(self,original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")
            
        return targetlocation


    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


    def get_bearing(self,aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """	
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def go_in_x_cm(self,cm):
        self.x = self.x + cm/100
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        #wait until drone reaches the target
        while True:
            current_x = self.drone.location.local_frame.north
            if current_x > self.x-0.1 and current_x < self.x+0.1:
                break
            time.sleep(0.5)
            
    def go_in_y_cm(self,cm):
        self.y = self.y + cm/100
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        
    def go_in_z_cm(self,cm):
        self.z = self.z + cm/100
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        
    def go_in_x_y_z_cm(self,x_cm,y_cm,z_cm):
        self.x = self.x + x_cm/100
        self.y = self.y + y_cm/100
        self.z = self.z + z_cm/100
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        
    def go_in_x_m(self,m):
        self.x = self.x + m
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        #wait until drone reaches the target
        while True:
            current_x = self.drone.location.local_frame.north
            if current_x > self.x-0.1 and current_x < self.x+0.1:
                break
            time.sleep(0.5)
            
    def go_in_y_m(self,m):
        self.y = self.y + m
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        #wait until drone reaches the target
        while True:
            current_y = self.drone.location.local_frame.east
            if current_y > self.y-0.1 and current_y < self.y+0.1:
                break
            time.sleep(0.5)
            
    def go_in_z_m(self,m):
        self.z = self.z + m
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        #wait until drone reaches the target
        while True:
            current_z = self.drone.location.local_frame.down
            if current_z > self.z-0.1 and current_z < self.z+0.1:
                break
            time.sleep(0.5)
            
    def go_in_x_y_z_m(self,x_m,y_m,z_m):
        self.x = self.x + x_m
        self.y = self.y + y_m
        self.z = self.z + z_m
        self.goto_position_target_local_ned(self.x,self.y,-self.z)
        
    def scan_rectangle_m(self,x,y):
        self.go_in_y_m(y)
        self.go_in_x_m(x)
        self.go_in_y_m(-2*y)
        self.go_in_x_m(-2*x)
        self.go_in_y_m(2*y)
        self.go_in_x_m(x)
        self.go_in_y_m(-y)
    

def main():
    control = Control()
    control.arm()
    control.takeoff(10)
    control.goto_position_target_local_ned(5,5,-15)
    control.goto_position_target_relative_ned(5,5,-1)
    control.send_ned_velocity(5,0,-1,5)
    control.condition_yaw(180, True) #turn 180 degrees
    control.turn_to_north()
    return
    
if __name__ == "__main__":
    main()