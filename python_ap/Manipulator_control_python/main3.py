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
robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
# robot.save_position()
robot.print()
robot.restore_position()
robot.print()
# file = open("lastPos", "w")
# file.truncate()
# file.write("ffff")
# file.close()
# # # robot.print()
# robot.calibrate('000010', '30')
# # robot.auto_calibrate()
# robot.print()
# # # robot.teensy_push("MJA10B17198C10D10E10F10S30G15H10I20K5")
# # robot.print()
# # print(robot.joints[0].motor_dir)
# # robot.joints[0].current_joint_angle = 170
#
# # R = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 2]])
# # logger.debug(R[2, 2])
# xyz = np.array([0.1, -0.5, 0.3, 0, pi, 0.])
# ang = robot.calculate_inverse_kinematic_problem(xyz)
# ang = np.degrees(ang)
# logger.debug(ang)
# massive = robot.calculate_direct_kinematics_problem()

# robot.auto_calibrate()
# # time.sleep(3)
# # robot.time_sleep = 1
# # # robot.print()
# # # robot.jog_joints([90, -34, 87, 0, -60, 0])
# # robot.read_points()
# # massive = robot.calculate_direct_kinematics_problem()
# logger.debug(massive)
# # robot.print()
# # robot.visual2()
# #robot.visual2()
# # logger.debug(robot.joints[1].current_joint_angle)
# robot.jog_joints([robot.joints[0].current_joint_angle, robot.joints[1].current_joint_angle, robot.joints[2].current_joint_angle, robot.joints[3].current_joint_angle, -90, robot.joints[5].current_joint_angle])
# # robot.jog_joints([ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]])
# robot.print()
# # robot.jog_joints([170, 0, 143, 164, 104, 148])
# robot.print()

# Отрицательный лимит 1го звена: -170
# Положительный лимит 1го звена: 170
#
#
# Отрицательный лимит 2го звена: -129.6
# Положительный лимит 2го звена: 0
#
#
# Отрицательный лимит 3го звена: 1
# Положительный лимит 3го звена: 143.7
#
#
# Отрицательный лимит 4го звена: -164.5
# Положительный лимит 4го звена: 164.5
#
#
# Отрицательный лимит 5го звена: -104.15
# Положительный лимит 5го звена: 104.15
#
#
# Отрицательный лимит 6го звена: -148.1
# Положительный лимит 6го звена: 148.1
#
#
# Введите команду
