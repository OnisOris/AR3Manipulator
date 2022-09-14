from manipulator import Manipulator
from config import DEFAULT_SETTINGS
import numpy as np

############## Настройки программы ##############
baud = 115200
teensy_port = 13
arduino_port = 10
################# Конец настроек #################

# пример команд
c1 = "MJA1444S25G15H10I20K5U0V2321W3X7599Y2281Z5313" + "\n"
c2 = "MJA13065B13762C13915D0577E0660F0148T10S25G15H10I20K5U10663V6084W3915X7023Y1627Z3164\n"
c3 = "MJA10B01109C11D00E00F01T10S25G15H10I20K5U10663V4975W3916X7023Y1627Z3163\n"
c4 = "MJA01775B11110C00D10E0218F00T10S25G15H10I20K5U8888V6085W3916X7023Y1409Z3163\n"

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
a = robot.matrix_dot(robot.matrix_create(), 0, 1)
print(a)
#robot.calibrate("010001", "40")
# if robot.is_connected:
#     robot.jog_joint(robot.joints[0], 20, 30)
# a = np.array([[2, 3],
#               [4, 6]])
# b = np.array([[43, 7],
#               [11, 19]])
# c = np.array([[24, 32],
#               [46, 65]])
#
# a2 = np.dot(a, b)
# f = np.dot(a2, c)
# print(f)
# for key, value in DEFAULT_SETTINGS.items():
#     print(f'{key} -- {value}')
T0_6 = robot.calculate_direct_kinematics_problem()
#print(T0_6)
#print(robot.calculate_inverse_kinematics_problem(robot.matrix_create()))


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
