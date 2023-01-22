import math
import time

from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger

############## Настройки программы ##############
baud = 115200
teensy_port = 6
arduino_port = 7
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
position = [68.944, 0.0, 733.607, -90.0, 1.05, -90.0]
robot.auto_calibrate()
robot.move_xyz(position)

robot.jog_joint(robot.joints[0], 30, 90)
robot.jog_joint(robot.joints[1], 30, 30)
robot.jog_joint(robot.joints[2], 30, 30)
#robot.jog_joint(robot.joints[3], 30, 30)

position = [0.157, 458.89, 545, -179, 63, -90.0]
robot.move_xyz(position)
#print(position+['F', 0, 0, 0, 0, 0, 0])

logger.debug(
    robot.calculate_inverse_kinematics_problem2(68.944, 0.0, 733.607, -90.0, 1.05, -90.0, 'F', 0, 0, 0, 0, 0, 0)
)

robot.finish()