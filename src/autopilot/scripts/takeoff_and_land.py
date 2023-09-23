from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--connect",default="127.0.0.1:14550")
args = parser.parse_args()

print(f"Connecting to vehicle {args.connect}")
vehicle = connect(args.connect, wait_ready=False,baud=57600)


def takeoff(drone,altitude):
    
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
    
    drone.simple_takeoff(altitude)
    while drone.location.global_relative_frame.alt < altitude*0.9:
        print("Taking off")
        time.sleep(1)
    
def rtl(drone):
    drone.mode = VehicleMode("RTL")
    
def land(drone):
    drone.mode = VehicleMode("LAND")

def main():

    takeoff(vehicle,10)
    loc = LocationGlobalRelative(-35.36329561, 149.16479801 ,20)
    vehicle.simple_goto(loc)
    time.sleep(10)
    rtl(vehicle)
    
if __name__ == "__main__":
    main()