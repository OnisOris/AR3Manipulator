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
arduino_port = 6
################# Конец настроек #################
r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud, camera=False, controller_dualshock=False, continuouse_mesurement=False)
# cap = cv2.VideoCapture(0)
#r.auto_calibrate()
r.rotate_relative([100, 100, 100, 100, 100, 100])

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)