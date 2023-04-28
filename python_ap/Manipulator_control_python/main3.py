import math
import time
from config import DEFAULT_SETTINGS
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard
#import keyboard
import matplotlib.pyplot as plt

############## Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 6
################# Конец настроек #################
robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)

robot.auto_calibrate()

xpoints = np.array([0.2, 0.2, 0.3, 0.4, 0.45])
ypoints = np.array([0.2, 0, -0.2, 0.3, 0.1])

for i in range(len(xpoints)):
    inv = robot.calculate_inverse_kinematic_problem([xpoints[i], ypoints[i], 0.3, 0, pi, 0])
# inv = robot.calculate_inverse_kinematic_problem(
#     [float(inp_c[1]) / 1000, float(inp_c[2]) / 1000, float(inp_c[3]) / 1000, np.radians(float(inp_c[4])),
#      np.radians(float(inp_c[5])), np.radians(float(inp_c[6]))])
# logger.debug(inv)
    ang = np.degrees(inv)
    robot.jog_joints([ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]])
fig = plt.figure()
ax = fig.add_subplot()
ax.set_xlabel('x')
ax.set_ylabel('y')
plt.plot(xpoints, ypoints)
plt.show()
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
