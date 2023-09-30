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
        
    
    
    def add_waypoint(self,latitude,longitude,altitude):
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,latitude,longitude,altitude))
    def rtl(self):
        self.command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))
    def scan_rectangle_m(self,x,y):
        self.control.go_in_y_m(y)
        self.control.go_in_x_m(x)
        self.control.go_in_y_m(-2*y)
        self.control.go_in_x_m(-2*x)
        self.control.go_in_y_m(2*y)
        self.control.go_in_x_m(x)
        self.control.go_in_y_m(-y)
    def field_scan(self):
        if(self.control.arm()):
            altitude = 4
            time.sleep(2)
            self.control.takeoff(altitude)
            time.sleep(2)
            self.drone.mode = VehicleMode("GUIDED")
            self.scan_rectangle_m(10,10)
            self.drone.mode = VehicleMode("RTL")
            print("Returning to launch")
            time.sleep(10)
            self.drone.close()
            print("Drone closed")
if __name__ == "__main__":
    drone = connect('127.0.0.1:14550', wait_ready=False)
    field_scan = FieldScan(drone)
    field_scan.field_scan()