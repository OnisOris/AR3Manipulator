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


