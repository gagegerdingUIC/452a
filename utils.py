import cv2
import numpy as np
import math

# ======== TO DO ========
#Fill out each functions based on the materials from the lecture.

def vec2hat(x):
    # Find x^
    x_hat = np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
    return x_hat

def cvdata2transmtx(rvec,tvec):
    # Rotation and translation of Camera to ArUco
    R_aruco, _ = cv2.Rodrigues(rvec)  # Convert rotation vector to rotation matrix
    p_aruco = tvec.reshape((3,1))
    
    # Find the Rotation and translation of ArUco to Camera (Inverse transformation)
    R_camera = R_aruco.T  # Transpose of the rotation matrix
    p_camera = -np.dot(R_camera, p_aruco)
    
    # Homogeneous transformation matrix
    g = np.eye(4)
    g[:3, :3] = R_camera
    g[:3, 3] = p_camera.flatten()
    
    return g, R_camera, p_camera

def transmtx2twist(g):
    # Rotation and translation from g
    R = g[:3, :3]
    p = g[:3, 3].reshape((3,1))
    
    # Convert the rotation matrix to rotation vector (including theta)
    rvec, _ = cv2.Rodrigues(R)
    
    # Find the twist coordinate
    th = np.linalg.norm(rvec)
    w = rvec / th
    v = np.dot(-vec2hat(w), p).flatten()
    
    return v, w, th

def twist2screw(v,w,th):
    # Convert the twist coordinate to screw motion
    q = w
    h = v / np.linalg.norm(w)
    u = th * w
    M = np.eye(4)
    M[:3, :3] = np.cross(vec2hat(w), h.reshape((3,1))) + np.dot(h, h.T) * np.cos(th)
    M[:3, 3] = (np.eye(3) * th + (1 - np.cos(th)) * vec2hat(w) + (th - np.sin(th)) * np.dot(vec2hat(w), vec2hat(w))) @ v

    return q, h, u, M

def distance(xdiff,zdiff):
    # Find the distance using delta(x) and delta(z) in the x-z plane
    dist = math.sqrt(xdiff**2 + zdiff**2)
    return dist
