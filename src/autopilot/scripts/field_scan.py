#!/usr/bin/env python3
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import dronekit
import time
from pymavlink import mavutil
import movement
class FieldScan():
    def __init__(self,drone:dronekit.Vehicle):
        self.drone = drone
        self.control = movement.Control(self.drone)
        self.command = self.drone.commands
        self.command.clear()
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
    def takeoff(self,altitude):
        self.drone.simple_takeoff(altitude)
        current_altitude = self.drone.location.global_relative_frame.alt
        while current_altitude < altitude*0.9:
            current_altitude = self.drone.location.global_relative_frame.alt
            print(f"Taking off ({current_altitude}m)")
            time.sleep(1)
    def add_waypoint(self,latitude,longitude,altitude):
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,latitude,longitude,altitude))
    def rtl(self):
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))
    def upload(self):  
        self.command.upload()
        print("Commands uploading...")
    def go_in_x_cm(self,cm):
        self.control.goto_position_target_relative_ned(cm,0,0)
    def go_in_y_cm(self,cm):
        self.control.goto_position_target_relative_ned(0,cm,0)
    def go_in_z_cm(self,cm):
        self.control.goto_position_target_relative_ned(0,0,-cm)
    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.
        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
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
        for x in range(0,duration):
            self.drone.send_mavlink(msg)
            time.sleep(1)
    def goto_position_target_relative_ned(self,north, east, down):
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
    def field_scan(self):
        if(self.arm()):
            time.sleep(2)
            self.takeoff(10)
            time.sleep(10)
            self.drone.mode = VehicleMode("GUIDED")
            time.sleep(2)
            self.control.goto_position_target_local_ned(0,4.5,-10)
            time.sleep(10)
            self.control.goto_position_target_local_ned(4.5,4.5,-10)
            time.sleep(10)
            self.control.goto_position_target_local_ned(4.5,-4.5,-10)
            time.sleep(10)
            self.control.goto_position_target_local_ned(0,-4.5,-10)
            time.sleep(10)
            self.control.goto_position_target_local_ned(0,0,-10)
            time.sleep(10)
            self.drone.mode = VehicleMode("LAND")
            print("Returning to launch")
            time.sleep(10)
            self.drone.close()
            print("Drone closed")
if __name__ == "__main__":
    drone = connect('127.0.0.1:14550', wait_ready=False)
    field_scan = FieldScan(drone)
    field_scan.field_scan()