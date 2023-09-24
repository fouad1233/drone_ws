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
        
    
    def takeoff(self,altitude):
        self.control.z = altitude
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
    def field_scan(self):
        if(self.control.arm()):
            altitude = 4
            time.sleep(2)
            self.takeoff(altitude)
            time.sleep(2)
            self.drone.mode = VehicleMode("GUIDED")
            self.control.go_in_y_m(5)
            self.control.go_in_x_m(5)
            self.control.go_in_y_m(-10)
            self.drone.mode = VehicleMode("RTL")
            print("Returning to launch")
            time.sleep(10)
            self.drone.close()
            print("Drone closed")
if __name__ == "__main__":
    drone = connect('127.0.0.1:14550', wait_ready=False)
    field_scan = FieldScan(drone)
    field_scan.field_scan()