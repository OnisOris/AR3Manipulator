import numpy as np
from manipulator import Manipulator, Position
import math
from math import (sin, cos, pi, atan2, sqrt, radians)

############# Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 6
################# Конец настроек #################

r = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
ijk = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])

#ijk = r.rotate_3(ijk, ijk[0], pi/3)
r.display_axis(ijk)
# print(r.RXY_transform())
ijk2 = ijk.dot(r.RXY_transform([pi/2, pi, 0]))
r.display_axis(ijk2)

ijk2 = ijk2.dot(r.RXY_transform([0, -pi/2, 0]))
r.display_axis(ijk2)

ijk2 = ijk2.dot(r.RXY_transform([pi/2, pi, 0]))
r.display_axis(ijk2)