import math
import time
from config import DEFAULT_SETTINGS
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
#logger.debug(robot.calculate_direct_kinematics_problem2(8.391331785652922e-05, -89.99514827193128, 1.0385111119522037, 0.013274359904453395, 0.006637160177256035, -0.01319046058787876))
# robot.joints[0].current_joint_angle = 170
# robot.joints[1].current_joint_angle = -129.6
# robot.joints[2].current_joint_angle = 143.7
# robot.joints[3].current_joint_angle = -164.5
# robot.joints[4].current_joint_angle = -104.15
# robot.joints[5].current_joint_angle = 148.1
# robot.print()
#print(DEFAULT_SETTINGS['DH_t_1'])
robot.auto_calibrate()
robot.print()
#robot.print()
# logger.debug(robot.joints[0].current_joint_angle)
# robot.jog_joint(robot.joints[0], 30, 40)
# logger.debug(robot.joints[0].current_joint_angle)
#robot.jog_joint(robot.joints[1], 30, 30)
#robot.jog_joint(robot.joints[2], 30, 30)
#robot.jog_joint(robot.joints[0], 30, 90)
pos = robot.print()
position = [161.29, 15.261, 713.24, -81.03, 44.104, -88.58]

robot.move_xyz(position)

position = [50, 15.261, 713.24, -81.03, 44.104, -88.58]

robot.move_xyz(position)
#print(position+['F', 0, 0, 0, 0, 0, 0])

# logger.debug(
#     robot.calculate_inverse_kinematics_problem2(68.944, 0.0, 733.607, -90.0, 1.05, -90.0, 'F', 0, 0, 0, 0, 0, 0)
# )

#robot.finish()