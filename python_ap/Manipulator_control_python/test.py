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
baud = 115200
teensy_port = 3
arduino_port = 6
################# Конец настроек #################
r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
r.print()
r.joints[0].current_joint_angle = 0
r.joints[1].current_joint_angle = -90
r.joints[2].current_joint_angle = 0
r.joints[3].current_joint_angle = 0
r.joints[4].current_joint_angle = 0
r.joints[5].current_joint_angle = 30



r.calculate_direct_kinematics_problem()


r.print()

logger.debug(r.last_matrix[1])

m02 = r.matrix_dot(r.last_matrix, 0, 6)
logger.debug(m02)

ijk = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

r.display_axis(ijk)

ijk = ijk.dot(m02[0:3, 0:3])

print(ijk)

r.display_axis(ijk)

print(r.joints[1].motor_direction)