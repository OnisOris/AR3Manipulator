import math
import time
from config import DEFAULT_SETTINGS
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard

############## Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 6
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
while (True):
    inp = input("Введите команду \n")
    inp_c = inp.split()
    try:
        if(inp ==  "exit"):
            break
        elif(inp ==  "calib"):
            robot.auto_calibrate()
        elif (inp_c[0] == "jog"):
            if (int(inp_c[1]) >= 1 and int(inp_c[1]) <= 6):
                robot.jog_joint(robot.joints[int(inp_c[1])-1], 20,  int(inp_c[2]))
            else:
                print("Звеньев все 6, введите число от 1 до 6")
        elif(inp ==  "help"):
            print("move_x [расстояние в мм] - передвижение по оси x в [мм]\n ")
            print("move_y [расстояние в мм] - передвижение по оси y в [мм]\n ")
            print("move_z [расстояние в мм] - передвижение по оси z в [мм]\n ")
            print("calib - автокалибровка\n ")
            print("jog [номер джойнта 1-6]\n ")
            robot.info()
        elif (inp_c[0] == "move_x"):
                robot.move_x(int(inp_c[1]))
        elif (inp_c[0] == "move_y"):
                robot.move_y(int(inp_c[1]))
        elif (inp_c[0] == "move_z"):
                robot.move_z(int(inp_c[1]))
        elif (inp_c[0] == "servo"):
                command = f"SV{0}P{inp_c[1]}\n"
                print(command)
                robot.arduino_push(command)
        elif (inp_c[0] == "grab"):
                robot.grab()
        elif (inp_c[0] == "absolve"):
                robot.absolve()
        elif (inp_c[0] == "move"):
                robot.jog_joint_c(robot.joints[int(inp_c[1])-1], int(inp_c[2]))
        elif (inp_c[0] == "print"):
                robot.print()
        elif (inp_c[0] == "move_all"):
                robot.jog_joints([inp_c[1], inp_c[2], inp_c[3], inp_c[4], inp_c[5], inp_c[6]])
        elif (inp_c[0] == "add"):
                robot.points += f"{robot.joints[0].current_joint_angle},{robot.joints[1].current_joint_angle},{robot.joints[2].current_joint_angle},{robot.joints[3].current_joint_angle},{robot.joints[4].current_joint_angle},{robot.joints[5].current_joint_angle}\n"
        elif (inp_c[0] == "save"):
                robot.write_point(robot.points)
        elif (inp_c[0] == "txmove"):
            ang = np.degrees(robot.calculate_inverse_kinematic_problem(
                [float(inp_c[1]), float(inp_c[2]), float(inp_c[3]), robot.joints[3].current_joint_angle,
                 robot.joints[4].current_joint_angle, robot.joints[5].current_joint_angle], True))
            robot.jog_joints(
                [ang[0], ang[1], ang[2], robot.joints[3].current_joint_angle, robot.joints[4].current_joint_angle,
                 robot.joints[5].current_joint_angle])
        else:
            print("Неправильная команда")
    except ValueError:
        print("Из этого положения нельзя передвинуться в новое")

# move_all 68 -34 87 0 -60 0
#position = [68.944, 0.0, 733.607, -90.0, 1.05, -90.0]
#logger.debug(robot.calculate_direct_kinematics_problem2(8.391331785652922e-05, -89.99514827193128, 1.0385111119522037, 0.013274359904453395, 0.006637160177256035, -0.01319046058787876))
# robot.joints[0].current_joint_angle = 170
# robot.joints[1].current_joint_angle = -129.6
# robot.joints[2].current_joint_angle = 143.7
# robot.joints[3].current_joint_angle = -164.5
# robot.joints[4].current_joint_angle = -104.15
# robot.joints[5].current_joint_angle = 148.1
# robot.print()
#print(DEFAULT_SETTINGS['DH_t_1'])
#robot.auto_calibrate()
##robot.print()
#robot.print()
# logger.debug(robot.joints[0].current_joint_angle)
# robot.jog_joint(robot.joints[0], 30, 40)
# logger.debug(robot.joints[0].current_joint_angle)
#robot.jog_joint(robot.joints[1], 30, 30)
#robot.jog_joint(robot.joints[2], 30, 30)
#robot.jog_joint(robot.joints[0], 30, 90)
#pos = robot.print()
#position = [161.29, 15.261, 713.24, -81.03, 44.104, -88.58]

# robot.move_xyz(position)
#robot.print()
# time.sleep(2)
# position = [299.45, 5.4327, 329.97, -98.96, 74, -82.43]
#
# robot.print()
# print("----------------------------")
# robot.jog_joint(robot.joints[0], "30", 90)
# robot.jog_joint(robot.joints[1], "30", 30)
# robot.jog_joint(robot.joints[2], "30", 30)
# robot.print()
#print("----------------------------")
# robot.move_z(20)
# robot.print()
# print("----------------------------")
#  need_angles=(90.0, -54.4326657322994, 14.139687841074192, -1.0585094009335781e-14, 11.29297789122522, 0.0)
# 2023-01-31 16:01:58.363 | DEBUG    | manipulator:move_xyz:456 - MJA10B1309C0943D00E1247F10S30G15H10I20K5
# 2023-01-31 16:01:58.364 | DEBUG    | manipulator:print:1137 - x = 0.15468345111515427 y = 443.13542747242195 z = 579.3600934009104 theta = -179.98 phi = 61.019999999999996 psi = -90.00000000000001
# robot.move_xyz([-1.337, 399.893, 329.457, -179.98, 74.28, -100.059])
# robot.move_z(-30)
# robot.print()

#robot.jog_joint(robot.joints[0], "30", 45)
#robot.move_xyz(position)
# robot.move_x(40)
# robot.print

# position = [50, 15.261, 713.24, -81.03, 44.104, -88.58]
#
# robot.move_xyz(position)
#print(position+['F', 0, 0, 0, 0, 0, 0])

# logger.debug(
#     robot.calculate_inverse_kinematics_problem2(68.944, 0.0, 733.607, -90.0, 1.05, -90.0, 'F', 0, 0, 0, 0, 0, 0)
# )

#robot.finish()