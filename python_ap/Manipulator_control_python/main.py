from manipulator import Manipulator

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

robot = Manipulator(teensy_port, arduino_port, baud, 6)
robot.jointsList[0].set_name_joint("A")
robot.jointsList[1].set_name_joint("B")
robot.jointsList[2].set_name_joint("C")
robot.jointsList[3].set_name_joint("D")
robot.jointsList[4].set_name_joint("E")
robot.jointsList[5].set_name_joint("F")
J1StepCur = 1
J1AngCur = 2
calibration = {'J1StepCur': 1, 'J1AngCur': 2}
print('Значение в массиве равно ' + str(calibration['J1StepCur']))
print(robot.jointsList[5].get_name_joint())
#robot.Jjog()
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