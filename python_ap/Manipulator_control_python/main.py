from manipulator import Manipulator

############## Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 5
################# Конец настроек #################

# пример команд
c1 = "MJA1444S25G15H10I20K5U0V2321W3X7599Y2281Z5313" + "\n"
c2 = "MJA13065B13762C13915D0577E0660F0148T10S25G15H10I20K5U10663V6084W3915X7023Y1627Z3164\n"
c3 = "MJA10B01109C11D00E00F01T10S25G15H10I20K5U10663V4975W3916X7023Y1627Z3163\n"
c4 = "MJA01775B11110C00D10E0218F00T10S25G15H10I20K5U8888V6085W3916X7023Y1409Z3163\n"

robot = Manipulator(teensy_port, arduino_port, baud)
print(robot.joints[0].current_joint_step)
robot.joints[0].current_joint_step = 100
print(robot.joints[0].current_joint_step)
# if robot.serial_arduino and robot.serial_teensy:
#     robot.move_to(c1)