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
teensy_port = 5
arduino_port = 4
################# Конец настроек #################
robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
robot.auto_calibrate()
robot.jog_joints([170, 0, 143, 164, 104, 148])
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
