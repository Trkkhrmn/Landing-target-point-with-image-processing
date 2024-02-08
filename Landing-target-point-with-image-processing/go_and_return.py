import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 

def arm_and_takeoff(aTargetAltitude):
    print ("Basic pre-arm checks")
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")

    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 
    
    while True:
        print (" Altitude: "), vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_global_int(aLocation):
    
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
        0b0000111111111000, 
        aLocation.lat*1e7, 
        aLocation.lon*1e7, 
        aLocation.alt, 
        0, 
        0, 
        0, 
        0, 0, 0, 
        0, 0)     

    vehicle.send_mavlink(msg)

def send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
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
print (" Mode: %s" % vehicle.mode.name)    # settable

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

print (" Home Location: %s" % vehicle.home_location)

vehicle.home_location=vehicle.location.global_frame
print (" New Home Location: %s" % vehicle.home_location)

arm_and_takeoff(1)

velocity_x = 0
velocity_y = 1
velocity_z = 0
duration = 10
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

vehicle.mode = VehicleMode("LAND")
arm_and_takeoff(1)
goto_position_target_global_int(vehicle.home_location)
vehicle.mode = VehicleMode("LAND")
disarm(wait=True, timeout=None)
vehicle.close()