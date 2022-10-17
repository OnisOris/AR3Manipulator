import time

from manipulator import Manipulator
import numpy as np
from math import (pi)

############## Настройки программы ##############
baud = 115200
teensy_port = 11
arduino_port = 13
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
# print(robot.calculate_inverse_kinematic_problem([[0.1],
#                                                  [0.1],
#                                                  [0.1]]))
robot.joints[0].current_joint_angle = 40
robot.joints[1].current_joint_angle = 50#-90.01
robot.joints[2].current_joint_angle = 5#1.05
robot.joints[3].current_joint_angle = 6.0
robot.joints[4].current_joint_angle = 50
robot.joints[5].current_joint_angle = 30
#robot.calibrate('100000', '10')
#time.sleep(3)
#robot.jog_joint(robot.joints[0], 20, 90)

#[178.96 180.   180.  ]
# print(robot.matrix_dot_all(robot.matrix_create()))
# print("----------------------------------------------")
# print(robot.matrix_dot(robot.matrix_create(), 0, 6))

#robot.matrix_create()
#print(np.around(robot.matrix_create()[5], 5))
#print(np.around(np.dot(robot.angular_Euler_calculation(robot.matrix_dot(robot.matrix_create(), 0, 6)), 180/pi), 3))
#print(robot.take_coordinate(robot.matrix_create(), 0, 6))
#print('------------------')
coordinate_array = np.array(robot.take_coordinate(robot.matrix_create(), 0, 6))
coordinate_array = np.array([[coordinate_array[0]], [coordinate_array[1]], [coordinate_array[2]]])

#print(coordinate_array)

#print('------------------')

print(f'Координаты нужных углов {np.round(np.dot(robot.calculate_inverse_kinematic_problem(coordinate_array), 180/pi), 3)}')
#print(robot.angular_Euler_calculation(robot.matrix_dot_all(robot.matrix_create())))
# print("-------------------------------------------------------------------")
#print(np.around(robot.matrix_create()[5], 5))
#print(robot.matrix_dot_all(robot.matrix_create()))
#robot.jog_joint(robot.joints[1], 10, 10)
# robot.calibrate("100000", "20")
#
# robot.jog_joint(robot.joints[1], 10, 10)
# robot.calibrate("010000", "20")
#
# robot.jog_joint(robot.joints[2], 10, 10)
# robot.calibrate("001000", "20")
#
# robot.jog_joint(robot.joints[3], 10, 10)
# robot.calibrate("000100", "20")
#
# robot.jog_joint(robot.joints[4], 10, 10)
# robot.calibrate("000010", "20")
#
# robot.jog_joint(robot.joints[5], 10, 20)
# robot.calibrate("000001", "20")

#robot.auto_calibrate()
#robot.jog_joint(robot.joints[2], 20, 20)



# print(robot.matrix_dot_all(robot.matrix_create()))
# print("-----------------")
# print(robot.matrix_dot(robot.matrix_create(), 0, 6))

# print(robot.matrix_dot_all(robot.matrix_create()))
# print("-----------------")
#print(robot.calculate_direct_kinematics_problem())


robot.serial_teensy.close()
robot.serial_arduino.close()



