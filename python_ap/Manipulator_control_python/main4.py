import math
import time
from config import DEFAULT_SETTINGS
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard
import cv2

############## Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 5
################# Конец настроек #################
r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
# ang = r.calculate_inverse_kinematic_problem([0.2, 0.0, 0.2, 0.0, pi, 0.0]) txmove -520 200 150 180 180 0
# txmove -515 200 150 180 180 0
# logger.debug(ang)
r.read_points()
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)