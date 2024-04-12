import cv2 as cv2
import numpy as np
import math
import matplotlib as plt


# ======== TO DO ========
# Fill out each function based on the lecture materials.

def vec2hat(x):
    # Function to create the skew-symmetric matrix (x^) from a vector x
    x_hat = np.array([[0, -x[2], x[1]],
                      [x[2], 0, -x[0]],
                      [-x[1], x[0], 0]],dtype=object,)
    return x_hat

def cvdata2transmtx(rvec, tvec):
    
    # Function to compute the homogeneous transformation matrix from camera to ArUco
    R_aruco, _ = cv2.Rodrigues(rvec)  # Convert rotation vector to rotation matrix
    p_aruco = tvec.reshape((3, 1))
    
    # Find the rotation and translation from ArUco to camera (inverse transformation)
    R_camera = R_aruco.T  # Transpose of the rotation matrix
    p_camera = -np.dot(R_camera, p_aruco)
    
    # Homogeneous transformation matrix
    g = np.eye(4)
    g[:3, :3] = R_camera
    g[:3, 3] = p_camera.flatten()
    
    return g, R_camera, p_camera

def transmtx2twist(g):
    # Function to compute the twist coordinates from the homogeneous transformation matrix
    R = g[:3, :3]
    p = g[:3, 3].reshape((3, 1))
    
    # Convert the rotation matrix to rotation vector (including theta)
    rvec, _ = cv2.Rodrigues(R)
    
    # Find the twist coordinate
    th = np.linalg.norm(rvec)
    w = rvec / th
    v = np.dot(-vec2hat(w), p).flatten()
    
    return v, w, th

def twist2screw(v, w, th):
    # Function to compute the screw motion from the twist coordinate
    q = np.cross(v, w,axis=0) # Point on screw axis
    h = np.linalg.norm(v) # Pitch
    u = w / np.linalg.norm(w) # Normalized w
    M = th # Magnitude

    return q, h, u, M

def distance(xdiff, zdiff):
    # Function to compute the distance using delta(x) and delta(z) in the x-z plane
    dist = math.sqrt(xdiff**2 + zdiff**2)
    return dist
