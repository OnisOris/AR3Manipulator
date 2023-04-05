import sys
import cv2
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
from math import sin,cos,radians
from numpy import dot
import arucoOdometry
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import pandas as pd
from scipy.signal import savgol_filter
import csv
aruco_marker_side_length = 0.0344
aruco_dictionary_name = "DICT_4X4_50"
# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboardDEXP1080.yaml'


# Start the video stream
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


odom = arucoOdometry.arucoOdometry()
odom.setCameraParams(camera_calibration_parameters_filename)
odom.setArucoLength(aruco_marker_side_length)
odom.setArucoDict(aruco_dictionary_name)
markers=[{"id":0,"size":aruco_marker_side_length},{"id":8,"size":aruco_marker_side_length}]
odom.setMarkers(markers)

startTime=time.time() * 1000
while(True):
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = cap.read()
    frame,x,y,z,a_x,a_y,a_z = odom.updateCameraPoses(frame,time.time()*1000-startTime,8)
    cv2.imshow("im",frame)
    cv2.waitKey(1)