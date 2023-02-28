import math
import time
from config import DEFAULT_SETTINGS
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard
#import keyboard

############## Настройки программы ##############
baud = 1152001
teensy_port = 3
arduino_port = 4
################# Конец настроек #################
robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
robot.joints[0].current_joint_angle = 10
robot.jog_joints(robot.joints[0], 30)

# robot.calc_angle(100.5, robot.joints[0])
# robot.auto_calibrate()