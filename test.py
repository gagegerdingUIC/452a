import RPi.GPIO as GPIO
import cv2
import numpy as np
import yaml
import utils
import math
import time
from picarx import Picarx

px = Picarx()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

def left_twitch(car):
    car.forward(0)
    car.set_dir_servo_angle(-15)
    car.forward(2)
    time.sleep(.2)
    car.forward(0)
    car.set_dir_servo_angle(0)
    time.sleep(.2)
def right_twitch(car):
    car.forward(0)
    car.set_dir_servo_angle(15)
    car.forward(2)
    time.sleep(.2)
    car.forward(0)
    car.set_dir_servo_angle(0)
    time.sleep(.2)
    
def forward_twitch(car):
    car.forward(2)
    time.sleep(.2)
    car.forward(0)

# Side length of the ArUco marker in meters 
marker_length = 0.033

# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L2)

goal_id = 0
helper1_id = 1
helper2_id = 2

print("Press q to save the transformation between Goal and Helper 1")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (len(corners)!=0):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i,_ in enumerate(rvecs):
                if ids[i] == goal_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc1 = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                    p = g_gc1[:,3]
                    p2 = p[[0,3]]
                    
                    mydist = np.linalg.norm(p2)
                    if (p[0]>0.2 and mydist>0.15):
                        left_twitch(px)
                    elif (p[0]<-0.2 and mydist>0.15):
                        right_twitch(px)
                    elif(mydist>0.2):
                        forward_twitch(px)
                    else:
                        px.forward(0)
                    
                    
                elif ids[i] == helper1_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch1 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])[0]
                    
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
