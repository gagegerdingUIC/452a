from picarx import Picarx

import time
import cv2
import numpy as np
import yaml
import utils
import math
import time




#======== TO DO ========
mindist= .1
px = Picarx()
turnTime = 4
speed =10

# define a list of functions that allows the robot to 
#turn 90 degree
#going forward/backward (or you can use the functions implemented in the examples)

def turn_left(car,angle):
    angle=np.abs(angle)
    car.forward(speed)
    car.set_dir_servo_angle((-angle)+20)
    time.sleep(turnTime)
    car.set_dir_servo_angle(0)

def turn_right(car,angle):
    angle=np.abs(angle)
    car.forward(speed)
    car.set_dir_servo_angle(angle-20)
    time.sleep(turnTime)
    car.set_dir_servo_angle(0)




#===================



# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()

# Side length of the ArUco marker in meters 
marker_length = 0.05
 
# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

print("Start running task 2...")

px.set_cam_pan_angle(-30)
time.sleep(1)
px.set_cam_pan_angle(0)
time.sleep(1)

# The width and height of the rectangle track
width = 0.8
height = 0.3

state_flag = 0 # flag to change between the two states
rot_flag = 0 # flag to check if the robot has rotated 90 degrees recently
count = 0 # flag to check whether it is operating with the current id of the ArUco marker

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        cv2.aruco.drawDetectedMarkers(frame,corners,ids,(0,225,0))
        if len(corners)!=0: # if aruco marker detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            g,_,p = utils.cvdata2transmtx(rvec,tvec)
            _,_,th = utils.transmtx2twist(g)
            cv2.imshow("aruco", frame)
            cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.05)
            if state_flag == 0 and count == ids:
                
                
                #======== TO DO ========
                #State 0. How should you set the goal points based on the moment when the robot 
                #detects a marker?
                goal_x = p[0]
                goal_z = p[2]
                
                state_flag = 1
                rot_flag = 0
                print("Goal point: x:{} z:{}".format(goal_x,goal_z))
                #=======================
                
                
            elif state_flag == 1 and count == ids:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                g,_,p = utils.cvdata2transmtx(rvec,tvec)
                xdiff = p[0] - goal_x
                zdiff = p[2] - goal_z
                cur_dist = utils.distance(xdiff,zdiff)
                while cur_dist > mindist:  # Move forward until close to the goal point
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                    g,_,p = utils.cvdata2transmtx(rvec,tvec)
                    _,_,th = utils.transmtx2twist(g)
                    xdiff = p[0] - goal_x
                    zdiff = p[2] - goal_z
                    cur_dist = utils.distance(xdiff,zdiff)
                    print("Not close enough, curdist: ", cur_dist)
                    px.forward(speed)
                else:  # Stop and rotate 90 degrees
                    print("I made it to the else")
                    px.forward(0)
                    time.sleep(1)
                    turn_left(px, 90)  # Rotate left by 90 degrees
                    state_flag = 0
                    rot_flag = 1
                    count += 1
    cv2.imshow('aruco',frame)

                #=======================
'''
        else:
            # If marker is missed
            if rot_flag == 1:
                turn_left(px, 90)  # Rotate left by 90 degrees
            else:
                if th > 0:  # If robot's orientation indicates forward direction
                    px.forward(speed)
                else:  # If robot's orientation indicates backward direction
                    px.forward(-speed)  
'''
            #=======================
       
#         cv2.imshow('aruco',frame)
#         key = cv2.waitKey(1500) & 0xFF
#         if key == ord('q'):
#             break
            
# Turn off the camera

cap.release()
cv2.destroyAllWindows()
