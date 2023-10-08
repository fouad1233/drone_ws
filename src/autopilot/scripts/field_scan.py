#!/usr/bin/env python3
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import dronekit
import time
from pymavlink import mavutil
import movement
import rospy
class FieldScan():
    def __init__(self,drone:dronekit.Vehicle):
        rospy.init_node('field_scan', anonymous=True)
        rospy.loginfo("Field scan node started")
        self.drone = drone
        self.control = movement.Control(self.drone)
        self.command = self.drone.commands
        self.command.clear()
    
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
            altitude = 1
            time.sleep(2)
            self.control.takeoff(altitude)
            time.sleep(2)
            self.drone.mode = VehicleMode("GUIDED")
            #self.scan_rectangle_m(10,10)
            self.control.go_in_y_m(10)
            #self.drone.mode = VehicleMode("RTL")
            #print("Returning to launch")
            #time.sleep(10)
            self.drone.close()
            #print("Drone closed")
def main():
    drone = connect('127.0.0.1:14550', wait_ready=False)
    
    field_scan = FieldScan(drone)
    field_scan.field_scan()
if __name__ == "__main__":
    main()