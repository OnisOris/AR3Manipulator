import math
import time

from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger

############## Настройки программы ##############
baud = 115200
teensy_port = 8
arduino_port = 7
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
robot.auto_calibrate()
logger.info(f'{robot.get_joints_angles()}')
# robot.move_xyz(Position(x=6.88455501e-02, y=-5.03769084e-17, z=7.33607349e-01, theta=2.980871355633929, phi=3.140331681299819, psi=0.3572458216613488))
# logger.debug(f"{robot.calculate_inverse_kinematic_problem()}")
# logger.debug(f"{robot.position}")

# direction = ['1', '1', '0', '1', '1', '0']
# steps = [7555, 2221, 7944, 7006, 2276, 3157]
# parameters = {
#     'speed': [50, 'S'],
#     'ACC_duration': [15, 'G'],
#     'ACC_speed': [10, 'H'],
#     'DEC_duration': [20, 'I'],
#     'DEC_speed': [5, 'K'],
# }
# param_command = []
# for param in parameters.values():
#     param_command.append(f"{param[1]}{param[0]}")
# parts = []
# for i, joint in enumerate(robot.joints):
#     parts.append(f"{joint.get_name_joint()}{direction[i]}{steps[i]}")
# my_command = f"MJ{''.join(parts)}{''.join(param_command)}\n"
# robot.teensy_push(my_command)
# logger.debug(f"Write to teensy: {my_command.strip()}")
# robot.serial_teensy.flushInput()
# robot.calculate_direct_kinematics_problem()
# logger.debug(f"{robot.position}")

robot.jog_joint(robot.joints[0], 30, 170)
robot.calculate_direct_kinematics_problem()
logger.info(robot.position)

robot.jog_joint(robot.joints[1], 30, 40)
robot.calculate_direct_kinematics_problem()
logger.info(robot.position)

robot.jog_joint(robot.joints[2], 30, -142)
robot.calculate_direct_kinematics_problem()
logger.info(robot.position)

robot.jog_joint(robot.joints[3], 30, 165)
robot.calculate_direct_kinematics_problem()
logger.info(robot.position)
#
robot.jog_joint(robot.joints[4], 30, 104)
robot.calculate_direct_kinematics_problem()
logger.info(robot.position)
#
robot.jog_joint(robot.joints[5], 30, -148)
robot.calculate_direct_kinematics_problem()
logger.info(robot.position)

# robot.jog_joint(robot.joints[1], 30, 20)
# robot.calculate_direct_kinematics_problem()
# logger.info(robot.position)
# logger.debug(f"{list(map(math.degrees, robot.calculate_inverse_kinematic_problem([[6.88455501e-02], [-5.03769084e-17], [7.33607349e-01]])))}")
logger.info(f'{robot.get_joints_angles()}')

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

robot.finish()



