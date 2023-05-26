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
#robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)

#robot.auto_calibrate()

xpoints = np.array([0.2, 0.2, 0.3, 0.4, 0.45])
ypoints = np.array([0.2, 0, -0.2, 0.3, 0.1])
zpoints = np.array([0.3, 0.4, 0.25, 0.3, 0.1])
massive = np.vstack([xpoints, ypoints])
massive = np.vstack([massive, zpoints])
logger.debug(massive)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_ylabel('z')
plt.plot(massive)
plt.show()