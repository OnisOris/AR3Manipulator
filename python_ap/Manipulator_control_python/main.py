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
#robot.joints[0].current_joint_angle = -180
# robot.joints[1].current_joint_angle = -45
#robot.joints[2].current_joint_angle = -150
#robot.joints[3].current_joint_angle = -180
# robot.joints[4].current_joint_angle = 50
# robot.joints[5].current_joint_angle = 120

#robot.jog_joint(robot.joints[0], 20, 180)
# robot.jog_joint(robot.joints[1], 20, 45)
#robot.jog_joint(robot.joints[2], 20, -60) #TODO: Разобраться с движениме в отрицательную сторону
#robot.jog_joint(robot.joints[3], 20, 180)
# robot.jog_joint(robot.joints[4], 20, 90)
# robot.jog_joint(robot.joints[5], 20, 180)

#robot.auto_calibrate()
#robot.move_xyz([0.068944, 0.05, 0.0])

#robot.calibrate('001000', '30')
#time.sleep(3)
#print(f'Координаты: {robot.calculate_direct_kinematics_problem()}')


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
#coordinate_array = np.array(robot.take_coordinate(robot.matrix_create(), 0, 6))
#coordinate_array = np.array([[coordinate_array[0]], [coordinate_array[1]], [coordinate_array[2]]])
#
#print(f'coordinate_array = {coordinate_array}')
#
#print(f'Координаты нужных углов {np.round(np.dot(robot.calculate_inverse_kinematic_problem(coordinate_array), 180/pi), 3)}')
# inverse = robot.calculate_inverse_kinematic_problem(coordinate_array)
# robot.joints[0].current_joint_angle = inverse[0]*180/pi
# robot.joints[1].current_joint_angle = inverse[1]*180/pi
# robot.joints[2].current_joint_angle = inverse[2]*180/pi
# robot.joints[3].current_joint_angle = inverse[3]*180/pi
# robot.joints[4].current_joint_angle = inverse[4]*180/pi
# robot.joints[5].current_joint_angle = inverse[5]*180/pi
# print(f'Координаты 2: {robot.calculate_direct_kinematics_problem()}')
#print(robot.angular_Euler_calculation(robot.matrix_dot_all(robot.matrix_create())))
# print("-------------------------------------------------------------------")
#print(np.around(robot.matrix_create()[5], 5))
#print(robot.matrix_dot_all(robot.matrix_create()))
# robot.jog_joint(robot.joints[0], 10, 10)
# for i in range(5):
#     robot.jog_joint(robot.joints[i], 10, 10)
#robot.calibrate("111111", "40")

#robot.calibrate("000001", "40")
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

# robot.jog_joint(robot.joints[0], 20, 180)
# robot.jog_joint(robot.joints[1], 20, 45)
# robot.jog_joint(robot.joints[2], 20, -150)
# robot.jog_joint(robot.joints[3], 20, 180)
# robot.jog_joint(robot.joints[4], 20, 90)
# robot.jog_joint(robot.joints[5], 20, 180)



# print(robot.matrix_dot_all(robot.matrix_create()))
# print("-----------------")
# print(robot.matrix_dot(robot.matrix_create(), 0, 6))

# print(robot.matrix_dot_all(robot.matrix_create()))
# print("-----------------")
#print(robot.calculate_direct_kinematics_problem())


robot.serial_teensy.close()
robot.serial_arduino.close()



