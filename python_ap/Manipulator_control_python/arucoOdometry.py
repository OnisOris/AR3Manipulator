import cv2
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
from math import sin,cos,radians
from numpy import dot
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import pandas as pd
from scipy.signal import savgol_filter
class arucoOdometry:
    arucoMarkers={"id":[],"coordinates":{"x":[],"y":[],"z":[],"theta":[],"phi":[],"psi":[]},"size":[]}
    #positionByMarkers={"id":[],"coordinates":{"x":[],"y":[],"z":[],"theta":[],"phi":[],"psi":[]}, "time":[]}
    #currentCoordinate={"x":0,"y":0,"z":0,"theta":0,"phi":0,"psi":0,"time":0}
    ARUCO_DICT = {
      "DICT_4X4_50": cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),#cv2.aruco.DICT_4X4_50,
      "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
      "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
      "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
      "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
      "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
      "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
      "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
      "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
      "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
      "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
      "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
      "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
      "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
      "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
      "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
      "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }

    
    aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    aruco_parameters = cv2.aruco.DetectorParameters_create()
    mtx=None
    dst=None
    aruco_length=0.031


    def __init__(self):
        pass
    def addMarker(self,marker):
        self.arucoMarkers["id"].append(marker["id"])
        self.arucoMarkers["size"].append(marker["size"])

    def setMarkers(self, markersList):
        for idx,marker in enumerate(markersList):
            self.addMarker(marker)
    def getMarker(self,markerId):
        markerIndex = self.arucoMarkers["id"].index(markerId)
        size=self.arucoMarkers["size"][markerIndex]
        marker={"id":markerId,"size":size}
        return marker
    def setCameraParams(self,camera_calibration_parameters_filename='calibration_chessboardDEXP1080.yaml'):
        cv_file = cv2.FileStorage(
            camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()
    def setArucoLength(self,value):
        self.aruco_length=value

    def setArucoDict(self,arucoDictName):
        if self.ARUCO_DICT.get(arucoDictName, None) is None:
          print("[INFO] ArUCo tag of '{}' is not supported".format(
            args["type"]))
          
        # Load the ArUco dictionary
        #print("[INFO] detecting '{}' markers...".format(
        #  aruco_dictionary_name))
        self.aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT[arucoDictName])
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()


    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
      
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
      
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
      
        return roll_x, pitch_y, yaw_z # in radians

    def updateCameraPoses(self,frame,frameTime,markerTargetID):   
        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.aruco_dictionary, parameters=self.aruco_parameters,
            cameraMatrix=self.mtx, distCoeff=self.dst)
        # Check that at least one ArUco marker was detected
        x,y,z,a_x,a_y,a_z=0,0,0,0,0,0
        if marker_ids is not None and markerTargetID in marker_ids:
 
          # Draw a square around detected markers in the video frame
          cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)

          
         
          # Print the pose for the ArUco marker
          # The pose of the marker is with respect to the camera lens frame.
          # Imagine you are looking through the camera viewfinder, 
          # the camera lens frame's:
          # x-axis points to the right
          # y-axis points straight down towards your toes
          # z-axis points straight ahead away from your eye, out of the camera
          for i, marker_id in enumerate(marker_ids):
            if(marker_id == markerTargetID):
                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.getMarker(marker_id)["size"],
                self.mtx,
                self.dst)

       
                # Store the translation (i.e. position) information
                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]
 
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                rotation_matrix[:3, 3]=np.array(tvecs[i][0])
                #rotation_matrix[0,3]
                rotation_matrix[1,3] -= 0.0327 #смещение носика дозатора
                rotation_matrix[2,3] -= 0.088 #смещение носика дозатора
                quat = r.as_quat()
                
                rInv = np.linalg.inv(rotation_matrix)
                camera_pos = rInv[:3, 3]

                # Углы Эйлера камеры в системе координат камеры
                R_camera_to_marker = rInv[:3,:3]
                camera_euler = R.from_matrix(R_camera_to_marker).as_euler('xyz', degrees=False)
         
                # Quaternion format     
                transform_rotation_x = quat[0] 
                transform_rotation_y = quat[1] 
                transform_rotation_z = quat[2] 
                transform_rotation_w = quat[3]
                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x,transform_rotation_y,transform_rotation_z,transform_rotation_w)

                x,y,z,a_x,a_y,a_z=transform_translation_x, transform_translation_y, transform_translation_z, roll_x, pitch_y, yaw_z
                print(camera_pos,camera_euler)
                x,y,z,a_x,a_y,a_z=camera_pos[0], camera_pos[1], camera_pos[2], camera_euler[0], camera_euler[1], camera_euler[2]





                # Draw the axes on the marker
                cv2.putText(frame,"x: {:.3f}".format(float(x)),[10, 450],cv2.FONT_HERSHEY_SIMPLEX,1,[50,50,255],2)
                cv2.putText(frame,"y: {:.3f}".format(float(y)),[210, 450],cv2.FONT_HERSHEY_SIMPLEX,1,[50,255,50],2)
                cv2.putText(frame,"z: {:.3f}".format(float(z)),[410, 450],cv2.FONT_HERSHEY_SIMPLEX,1,[255,50,50],2)
                cv2.putText(frame,"a_x: {:.1f}".format(float(math.degrees(a_x))),[10, 400],cv2.FONT_HERSHEY_SIMPLEX,1,[50,50,255],2)
                cv2.putText(frame,"a_y: {:.1f}".format(float(math.degrees(a_y))),[210, 400],cv2.FONT_HERSHEY_SIMPLEX,1,[50,255,50],2)
                cv2.putText(frame,"a_z: {:.1f}".format(float(math.degrees(a_z))),[410, 400],cv2.FONT_HERSHEY_SIMPLEX,1,[255,50,50],2)




                


                cv2.aruco.drawAxis(frame, self.mtx, self.dst, rvecs[i], tvecs[i], self.aruco_length)
        return frame, x,y,z,a_x,a_y,a_z
                


