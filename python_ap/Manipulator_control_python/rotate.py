import loguru
import numpy as np
from manipulator import Manipulator, Position
import math
from math import (sin, cos, pi, atan2, sqrt, radians)
from scipy.spatial.transform import Rotation

############# Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 6
################# Конец настроек #################

r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
#r.auto_calibrate()
v = np.array([0, 0, -1])
matrix_r = Rotation.from_rotvec(pi/3 * v/np.linalg.norm(v)).as_matrix()
ijk = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])  # 0
r.display_axis(ijk)
ijk = ijk.dot(matrix_r)
r.display_axis(ijk)
# v = np.array([0, 0, -1])
# matrix_r = Rotation.from_rotvec(0 * v/np.linalg.norm(v)).as_matrix()
# ijk = ijk.dot(matrix_r)
# r.display_axis(ijk)
# r.joints[0].current_joint_angle = 0
# mat = r.matrix_create()[0]
# loguru.logger.debug(mat[0:3, 0:3])
# ijk = ijk.dot(mat[0:3, 0:3])
#
# r.display_axis(ijk)
#ijk = r.rotate_3(ijk, ijk[0], pi/3)
ijk = ijk.dot(r.RXY_transform([pi/2, pi, 0])) # 1
r.display_axis(ijk)
# loguru.logger.debug(matrix_r)
# print(r.RXY_transform())
ijk2 = ijk.dot(r.RXY_transform([0, -pi/2, 0])) # 2
r.display_axis(ijk2)

ijk2 = ijk2.dot(r.RXY_transform([0, 0, -pi/2])) # 3
r.display_axis(ijk2)

ijk2 = ijk2.dot(r.RXY_transform([0, 0, pi/2]))# 4
r.display_axis(ijk2)

ijk2 = ijk2.dot(r.RXY_transform([0, 0, pi/2])) # 5
r.display_axis(ijk2)

ijk2 = ijk2.dot(r.RXY_transform([0, pi, 0])) # 6
r.display_axis(ijk2)