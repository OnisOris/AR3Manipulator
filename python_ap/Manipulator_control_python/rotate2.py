import loguru
import numpy as np
from manipulator import Manipulator, Position
import math
from math import (sin, cos, pi, atan2, sqrt, radians)
from scipy.spatial.transform import Rotation

############# Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 4
################# Конец настроек #################

r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
#r.auto_calibrate()
# v = np.array([0, 0, -1])
# matrix_r = Rotation.from_rotvec(pi/3 * v/np.linalg.norm(v)).as_matrix()
ijk = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]]) # 0
r.display_axis(ijk)
r.matrixRZYZ([0, 0, pi/3, 1, 1, 1])
# ijk = ijk.dot(r.RZYZ_transform([0, 0, pi/3]))
# r.display_axis(ijk)
