import sys
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard

############## Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 5
################# Конец настроек #################

r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
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
def trans(xyzabc):
    theta = xyzabc[5]+pi
    x = xyzabc[0]
    y = xyzabc[1]
    z = xyzabc[2]
    T6_7 = np.array([[cos(theta), -sin(theta), 0, x*cos(theta)],
          [sin(theta), cos(theta), 0, y*sin(theta)],
          [0, 0, 1, z],
          [0, 0, 0, 1]])
    T0_6 = r.matrix_dot(r.calculate_direct2(), 0, 6)
    angles = r.angular_Euler_calculation(T0_6[0:3, 0:3])
    #logger.debug(np.degrees(angles))
    T0_7 = np.dot(T0_6, T6_7)
    #logger.debug(T0_7)
    angles = r.angular_Euler_calculation(T0_7[0:3, 0:3])
    #logger.debug(np.degrees(angles))
    return [T6_7[0, 3], T6_7[1, 3], T6_7[2, 3]]
# trans()
# Start the video stream
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
logger.debug("cap.set")

odom = arucoOdometry.arucoOdometry()
odom.setCameraParams(camera_calibration_parameters_filename)
odom.setArucoLength(aruco_marker_side_length)
odom.setArucoDict(aruco_dictionary_name)
markers=[{"id": 10, "size": aruco_marker_side_length}, {"id": 6, "size": aruco_marker_side_length}]
odom.setMarkers(markers)

startTime=time.time() * 1000
r.move_xyz([0.28683, 0, 0.28977, 0, pi, 0])
while(True):
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = cap.read()
    frame,x,y,z,a_x,a_y,a_z = odom.updateCameraPoses(frame,time.time()*1000-startTime,10)
    cv2.imshow("im",frame)
    cv2.waitKey(1)
    if not x == 0 or not y == 0 or not z == 0:
        logger.debug(f'1 x = {x} y = {y} z = {z}')
        #xyz = trans([x, y, z, a_x, a_y, a_z])
        #logger.debug(f'2 ---- x = {xyz[0]} y = {xyz[1]} z = {xyz[2]}')
        # xyz[0] = -xyz[0]
        # xyz[1] = -xyz[1]
        # xyz[2] = -xyz[2]
       # k = 0.5
        #r.move_all_xyz([-xyz[0]*k, -xyz[1]*k, 0])
        #time.sleep(0.3)

# Fail to create pixmap with Tk_GetPixmap in TkImgPhotoInstanceSetSize


