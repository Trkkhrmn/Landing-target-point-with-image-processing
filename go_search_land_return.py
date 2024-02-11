import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 

target1 = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/VEFA_logo.png", cv2.IMREAD_GRAYSCALE)
target2 = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/YTU_logo.png", cv2.IMREAD_GRAYSCALE)
target4 = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/LAND_logo.png", cv2.IMREAD_GRAYSCALE)
home = cv2.imread("C:/Users/KAHRAMAN/Desktop/drone/TR_flag.png", cv2.IMREAD_GRAYSCALE)

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
        print( " Altitude: ", vehicle.location.global_relative_frame.alt)
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
def search_lock(target, algo):
    cap = cv2.VideoCapture(0)
    cap.set(3,432)
    cap.set(4,432)
    if(algo == "orb"):
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(target, None)
        output = cv2.drawKeypoints(target, keypoints, None)

        indexparameters= dict(algorithm = 6,
                     table_number = 12,
                     key_size = 20,
                     multi_probe_level = 2)

        searchparameters = dict(checks=30)
        flann = cv2.FlannBasedMatcher(indexparameters, searchparameters)

    if(algo == "surf"):
        surf = cv2.xfeatures2d.SURF_create() 
        keypoints, descriptors = surf.detectAndCompute(target, None) 
        output = cv2.drawKeypoints(target, keypoints, None)   

        indexparameters1 = dict(algorithm= 0 , trees = 5) 
        searchparameters1 = dict()
        flann = cv2.FlannBasedMatcher(indexparameters1, searchparameters1)

    
    lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    color = np.random.randint(0,255,(100,3))

    if cap.isOpened():
        ret , frame = cap.read()

    else:
        ret = False

    while ret:
        cv2.imshow("ORB-TARGET", output)

        ret , frame = cap.read()
        medianBlur = cv2.medianBlur(frame,5)
        grayFrame = cv2.cvtColor(medianBlur,cv2.COLOR_BGR2GRAY)   

        if (algo == "orb"):
            print("ORB ALGORITHM ")
            keypoints_grayFrame, descriptors_grayFrame = orb.detectAndCompute(grayFrame, None)
            show_keypoints_grayFrame = cv2.drawKeypoints(grayFrame,keypoints_grayFrame, None)

        if (algo == "surf"):
            print("SURF ALGORITHM ")
            keypoints_grayFrame, descriptors_grayFrame = surf.detectAndCompute(grayFrame, None)
            show_keypoints_grayFrame = cv2.drawKeypoints(grayFrame,keypoints_grayFrame, None)
            cv2.imshow("Real Time Cap surf", show_keypoints_grayFrame)

        matches_flann = flann.knnMatch(descriptors, descriptors_grayFrame, k=2)
        
        goodMatches_flann = []
        for m in matches_flann:
            if len(m) > 0 and m[0].distance < 0.2*m[-1].distance:
                goodMatches_flann.append(m[0])

        
        result_flann = cv2.drawMatches(target, keypoints, grayFrame, keypoints_grayFrame, goodMatches_flann, grayFrame)

        if len(goodMatches_flann) > 7:
            cv2.destroyWindow("Result")
            cv2.putText(result_flann,'TARGET-DETECTED',(650,100), cv2.FONT_HERSHEY_SIMPLEX, .5,(0,0,255),2,cv2.LINE_AA)
            cv2.imshow("Target-detected", result_flann)

            
            queryPoints = np.float32([keypoints[i.queryIdx].pt for i in goodMatches_flann]).reshape(-1, 1, 2)   
            trainPoints = np.float32([keypoints_grayFrame[j.trainIdx].pt for j in goodMatches_flann]).reshape(-1, 1, 2) 
            
            targetPoints= trainPoints

            mask = np.zeros_like(frame)

            while(len(goodMatches_flann) > 7):
                ret,frame2 = cap.read()
                newGrayFrame = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

                flowMatches, st, err = cv2.calcOpticalFlowPyrLK(grayFrame, newGrayFrame, targetPoints, None, **lk_params)

                try:
                    good_new = flowMatches[st==1]
                    good_old = targetPoints[st==1]
                except:
                    break

                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    print("new points",(a,b))
                    c,d = old.ravel()
                    print("old points",(c,d))
                    maskk= cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
                    frame3 = cv2.circle(frame2,(a,b),5,color[i].tolist(),-1)
                
                    if not((150<a and a<195) and (115<b and b<155)):  
                        if(a < 130):
                            print("RIGHT!")
                            send_body_ned_velocity(0.5, 0, 0, duration=1)
                        if(a > 220):
                            print("LEFT!")
                            send_body_ned_velocity(-0.5, 0, 0, duration=1)
                        if(b < 114):
                            print("BACK!")
                            send_body_ned_velocity(0, -0.5, 0, duration=1)
                        if(b > 190):
                            print("FRONT!")
                            send_body_ned_velocity(0, 0.5, 0, duration=1)
                    else:
                        cv2.putText(frame3,'Initializing Landing...',(80,80), cv2.FONT_HERSHEY_SIMPLEX, .5,(30,40,50),2,cv2.LINE_AA)
                        
                        print("Initiate Landing...")
                        return True

                img = cv2.add(frame3,maskk)
                img= cv2.rectangle(img, (100,95), (250,175),(255,0, 0), 2)
                img= cv2.rectangle(img, (120,115), (230,155),(0,255, 0), 2)    
                cv2.imshow('frame',img)
                grayFrame = newGrayFrame.copy()
                targetPoints = good_new.reshape(-1,1,2)
                if cv2.waitKey(1) == 27:
                    break
        else:
            cv2.destroyWindow("Target-detected")
            cv2.destroyWindow("frame")
            cv2.imshow("Result", result_flann)

        if cv2.waitKey(1) == 27:
                break

    cv2.destroyAllWindows()
    cap.release()

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

vehicle.home_location=vehicle.location.global_frame
print (" New Home Location: %s" % vehicle.home_location)

arm_and_takeoff(2.5)
velocity_x = 1
velocity_y = 1
velocity_z = 0
duration = 5
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)
if search_lock(target1, "algo", 0.5) == True:
    vehicle.mode = VehicleMode("LAND")
else:
    goto_position_target_global_int(vehicle.home_location)
    disarm(wait=True, timeout=None)
    vehicle.close()
    
arm_and_takeoff(2.5)
velocity_x = 0.65
velocity_y = 0.35
velocity_z = 0
duration = 5
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)
if search_lock(target1, "algo", 0.5) == True:
    vehicle.mode = VehicleMode("LAND")
    disarm(wait=True, timeout=None)
else:
    goto_position_target_global_int(vehicle.home_location)
    vehicle.mode = VehicleMode("LAND")
    disarm(wait=True, timeout=None)
    vehicle.close()

arm_and_takeoff(0.5)
velocity_x = -1.3
velocity_y = 0
velocity_z = -0.33
duration = 5
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)
vehicle.mode = VehicleMode("LAND")
disarm(wait=True, timeout=None)

arm_and_takeoff(0.5)
velocity_x = 0
velocity_y = -1.30
velocity_z = -0.24
duration = 5
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)
if search_lock(target3, "algo", 0.68) == True:
    vehicle.mode = VehicleMode("LAND")
    disarm(wait=True, timeout=None)
else:
    goto_position_target_global_int(vehicle.home_location)
    vehicle.mode = VehicleMode("LAND")
    disarm(wait=True, timeout=None)
    vehicle.close()

arm_and_takeoff(2.30)
velocity_x = 1.3
velocity_y = 0
velocity_z = 0
duration = 5
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)
if search_lock(target4, "algo", 0.6) == True:
    vehicle.mode = VehicleMode("LAND")
    disarm(wait=True, timeout=None)
else:
    goto_position_target_global_int(vehicle.home_location)
    vehicle.mode = VehicleMode("LAND")
    disarm(wait=True, timeout=None)
    vehicle.close()

arm_and_takeoff(0.5)
goto_position_target_global_int(vehicle.home_location)
vehicle.mode = VehicleMode("LAND")
disarm(wait=True, timeout=None)
vehicle.close()
