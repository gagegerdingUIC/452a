import os.path
import cv2
import numpy as np
import yaml
import utils
import math

# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

# Side length of the ArUco marker in meters 
marker_length = 0.06

# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L2)

goal_id = 0
helper1_id = 1
helper2_id = 2
helper3_id = 3
helper4_id = 4



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
                    g1_gc = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                elif ids[i] == helper1_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g1_ch1 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

print("Press q to save the transformation between Helper1 and Helper 2")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (len(corners)!=0):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i,_ in enumerate(rvecs):
                if ids[i] == helper1_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g2_h1c = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                elif ids[i] == helper2_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g2_ch2 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
print("Press q to save the transformation between Helper2 and Helper 3")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (len(corners)!=0):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i,_ in enumerate(rvecs):
                if ids[i] == helper2_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g3_h2c = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                elif ids[i] == helper3_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g3_ch3 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
print("Press q to save the transformation between Helper3 and Helper 4")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (len(corners)!=0):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i,_ in enumerate(rvecs):
                if ids[i] == helper3_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g4_h3c = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                elif ids[i] == helper4_id:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g4_ch4 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()


g_gh1 = g1_gc @ g1_ch1
g_h1h2 = g2_h1c @ g2_ch2
g_h2h3 = g3_h2c @ g3_ch3
g_h3h4 = g4_h3c @ g4_ch4

g_gh2 = g_h1h2 @ g_gh1
g_gh3 = g_h2h3 @ g_gh2
g_gh4 = g_h3h4 @ g_gh3




calib_data = {
    'camera_matrix':np.asarray(mtx).tolist(),
    'distortion_coefficients':np.asarray(dist).tolist(),
    'g_gh1':np.asarray(g_gh1).tolist(),
    'g_gh2':np.asarray(g_gh2).tolist(),
    'g_gh3':np.asarray(g_gh3).tolist(),
    'g_gh4':np.asarray(g_gh4).tolist()
}

path = r"/home/452Lab/picar-x/example/452/calib_data.yaml"
assert os.path.isfile(path)
with open(path, 'w') as file:
    yaml.dump(calib_data, file)
