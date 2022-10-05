from manipulator import Manipulator
from math import (pi)

############## Настройки программы ##############
baud = 115200
teensy_port = 11
arduino_port = 6
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
# print(robot.calculate_inverse_kinematic_problem([[0.1],
#                                                 [0.1],
#                                                 [0.1]]))
robot.joints[0].current_joint_angle = 0.0
robot.joints[1].current_joint_angle = -90.01
robot.joints[2].current_joint_angle = 1.05
robot.joints[3].current_joint_angle = 0.0
robot.joints[4].current_joint_angle = 0.0
robot.joints[5].current_joint_angle = 0
#robot.matrix_create()
print(robot.matrix_dot(robot.matrix_create(), 0, 5))
#print(robot.matrix_create()[5])
#print(robot.calculate_direct_kinematics_problem())
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
robot.serial_teensy.close()
robot.serial_arduino.close()



