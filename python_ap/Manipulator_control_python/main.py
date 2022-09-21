import math

from manipulator import Manipulator
from config import DEFAULT_SETTINGS
import numpy as np

############## Настройки программы ##############
baud = 115200
teensy_port = 13
arduino_port = 10
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
#robot.matrix_create()
#print(robot.calculate_inverse_kinematic_problem([[1],
  #                                               [34],
     #                                            [23]], robot.matrix_create()))
print(robot.matrix_create()[5])
# print(robot.calculate_direct_kinematics_problem())
# print(range(1))
# print(math.sin(math.pi/2))


