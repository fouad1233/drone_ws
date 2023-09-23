from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
from pymavlink import mavutil

#drone = connect('127.0.0.1:14550', wait_ready=False)

class Control:
    def __init__(self, drone):
        self.drone = drone

    def arm(self):
        if not self.drone.is_armable:
            print("Drone is not armable")
            time.sleep(1)
            return
        
        print("Drone is armable")
        self.drone.mode = VehicleMode("GUIDED")
        time.sleep(1)
        print(self.drone.mode)

        self.drone.armed = True
        while not self.drone.armed:
            print("Drone is getting armed")
            time.sleep(0.5)
        print("Drone armed")
        
    def takeoff(self, altitude):
        
        self.drone.simple_takeoff(altitude)
        current_altitude = self.drone.location.global_relative_frame.alt
        while current_altitude < altitude*0.9:
            current_altitude = self.drone.location.global_relative_frame.alt
            print(f"Taking off ({current_altitude}m)")
            time.sleep(1)
        
    def add_mission(self):
        global command
        command = self.drone.commands
        command.clear()
        time.sleep(1)
        
        altitude = 10
        #TAKE OFF
        command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,0,altitude))
        #WAYPOINT
        command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,-35.36262207,149.16511350,20))
        command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,-35.36322841, 149.16584325,30))
        #RTL
        command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))
        #Validation
        command.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))

        command.upload()
        print("Commands uploading...")

    def goto_position_target_local_ned(self,north, east, down):
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

    def turn_to_north(self,):
        self.condition_yaw(0, False)


def main():
    drone = connect('127.0.0.1:14550', wait_ready=False)
    control = Control(drone)
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