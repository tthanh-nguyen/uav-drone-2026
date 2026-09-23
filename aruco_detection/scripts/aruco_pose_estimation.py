import numpy as np 
import cv2 
import cv2.aruco as aruco
import sys, time, math


# Define Tag 
id_to_find = 1
marker_size = 10   # [cm]

# Get the camera calibration path 
calib_path = ""
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
