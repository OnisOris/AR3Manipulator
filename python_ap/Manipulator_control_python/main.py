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


# print('Значение в массиве равно ' + str(calibration['J1StepCur']))
# print(robot.joints[0].get_name_joint())
# robot.Jjog()
# try:
#     with open("./program_files/Robot_calibration_data.cal", "rb") as calibration_file:
#         calibration_data = pickle.load(calibration_file)
# except FileNotFoundError:
#     calibration_data = "0"
#     write_calibration_data(calibration_data)
#
# for item in calibration_data:
#     calibration.insert(tk.END, item)
# if robot.serial_arduino and robot.serial_teensy:
#     robot.move_to(c1)
