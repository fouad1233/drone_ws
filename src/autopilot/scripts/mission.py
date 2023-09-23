from dronekit import connect, VehicleMode, Command
import time
from pymavlink import mavutil

drone = connect('127.0.0.1:14550', wait_ready=False)

def arm():
    if not drone.is_armable:
        print("Drone is not armable")
        time.sleep(1)
        return
    
    print("Drone is armable")
    drone.mode = VehicleMode("GUIDED")
    time.sleep(1)
    print(drone.mode)

    drone.armed = True
    while not drone.armed:
        print("Drone is getting armed")
        time.sleep(0.5)
    print("Drone armed")
    

def takeoff(altitude):
    
    drone.simple_takeoff(altitude)
    current_altitude = drone.location.global_relative_frame.alt
    while current_altitude < altitude*0.9:
        current_altitude = drone.location.global_relative_frame.alt
        print(f"Taking off ({current_altitude}m)")
        time.sleep(1)
        

        
def add_mission():
    global command
    command = drone.commands
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

def main():
    arm()
    takeoff(10)
    add_mission()
    command.next = 0
    
    drone.mode = VehicleMode("AUTO")
    
    while True:
        next_waypoint = command.next
        print(f"Next mission number {next_waypoint}")
        time.sleep(1)
        
        if next_waypoint == 4:
            print("Mission over")
            break
    
    print("Döngüden çıkıldı")
            
    
if __name__ == "__main__":
    main()