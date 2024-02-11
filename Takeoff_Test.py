import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 

def disarm(wait=True, timeout=None):
    print("Disarming")
    vehicle.armed = False
    if wait:
        while vehicle.armed:
            time.sleep(1)
            if timeout is not None and timeout <= 0:
                break
            if timeout is not None:
                timeout -= 1

def arm_and_takeoff(aTargetAltitude):
    print ("Basic pre-arm checks")
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 

    while True:
        print (" Altitude: "), vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

vehicle = connect('/dev/ttyS0', baud=921600)

print (" Get some vehicle attribute values:")
print (" GPS: %s" % vehicle.gps_0)
print (" Battery: %s" % vehicle.battery)
print (" Attitude: %s" % vehicle.attitude)
print (" Velocity: %s" % vehicle.velocity)
print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
print (" Is Armable?: %s" % vehicle.is_armable)
print (" System status: %s" % vehicle.system_status.state)
print (" Mode: %s" % vehicle.mode.name)    

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

print (" Home Location: %s" % vehicle.home_location)

vehicle.home_location = vehicle.location.global_frame
print (" New Home Location: %s" % vehicle.home_location)

arm_and_takeoff(1)

vehicle.mode = VehicleMode("LAND")
disarm(wait=True, timeout=None)
vehicle.close()
