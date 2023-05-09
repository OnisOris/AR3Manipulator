import math
import time
from config import DEFAULT_SETTINGS
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard
import threading

############## Настройки программы ##############
baud = 115200
teensy_port = 9
arduino_port = 6
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud, controller_dualshock=True)
robot.start_program()
#monitor = threading.Thread(target=robot.monitorEnc, daemon=True)
#console = threading.Thread(target=robot.startConsole, daemon=True)
#robot.startConsole()
# monitor = threading.Thread(target=robot.monitorEnc(), daemon=True)
#console.start()
#
#console.join()
# while (True):
#     inp = input("Введите команду \n")
#     inp_c = inp.split()
#     if (True):
#         if(inp ==  "exit"):
#             break
#         elif(inp ==  "c"):
#             robot.auto_calibrate()
#         # elif (inp_c[0] == "jog"):
#         #     if (int(inp_c[1]) >= 1 and int(inp_c[1]) <= 6):
#         #         robot.jog_joints(robot.joints[int(inp_c[1])-1], 20,  int(inp_c[2]))
#         #     else:
#         #         print("Звеньев все 6, введите число от 1 до 6")
#         elif(inp ==  "help"):
#             print("move_x [расстояние в мм] - передвижение по оси x в [мм]\n ")
#             print("move_y [расстояние в мм] - передвижение по оси y в [мм]\n ")
#             print("move_z [расстояние в мм] - передвижение по оси z в [мм]\n ")
#             print("calib - автокалибровка\n ")
#             #print("jog [номер джойнта 1-6]\n ")
#             robot.info()
#         elif (inp_c[0] == "move_x"):
#                 robot.move_x(int(inp_c[1]))
#         elif (inp_c[0] == "move_y"):
#                 robot.move_y(int(inp_c[1]))
#         elif (inp_c[0] == "move_z"):
#                 robot.move_z(int(inp_c[1]))
#         elif (inp_c[0] == "servo"):
#                 command = f"SV{0}P{inp_c[1]}\n"
#                 print(command)
#                 robot.arduino_push(command)
#         elif (inp_c[0] == "g"):
#                 robot.grab()
#         elif (inp_c[0] == "a"):
#                 robot.absolve()
#         elif (inp_c[0] == "rot"):
#                 robot.jog_joint_c(int(inp_c[1]), int(inp_c[2]))
#         elif (inp_c[0] == "print"):
#                 robot.print()
#         elif (inp_c[0] == "move_all"):
#                 robot.jog_joints([inp_c[1], inp_c[2], inp_c[3], inp_c[4], inp_c[5], inp_c[6]])
#         elif (inp_c[0] == "add"):
#                 points = f"inv,{robot.last_inverse_pos[0]},{robot.last_inverse_pos[1]},{robot.last_inverse_pos[2]},{robot.last_inverse_pos[3]},{robot.last_inverse_pos[4]},{robot.last_inverse_pos[5]}\n"
#                 #logger.debug(robot.points)
#                 robot.write_point(points)
#                 #robot.points += f"{robot.joints[0].current_joint_angle},{robot.joints[1].current_joint_angle},{robot.joints[2].current_joint_angle},{robot.joints[3].current_joint_angle},{robot.joints[4].current_joint_angle},{robot.joints[5].current_joint_angle}\n"
#         elif (inp_c[0] == "add2"):
#                 points2 = f"dir,{robot.joints[0].current_joint_angle},{robot.joints[1].current_joint_angle},{robot.joints[2].current_joint_angle},{robot.joints[3].current_joint_angle},{robot.joints[4].current_joint_angle},{robot.joints[5].current_joint_angle}\n"
#                 robot.write_point(points2)
#         elif (inp_c[0] == "txmove"):
#             inv = robot.calculate_inverse_kinematic_problem(
#                 [float(inp_c[1])/1000, float(inp_c[2])/1000, float(inp_c[3])/1000,  np.radians(float(inp_c[4])),
#                  np.radians(float(inp_c[5])), np.radians(float(inp_c[6]))])
#             logger.debug(inv)
#             ang = np.degrees(inv)
#             robot.jog_joints([ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]])
#         elif (inp_c[0] == "vis"):
#             robot.show()
#         elif (inp_c[0] == "vis_on"):
#             robot.showMode = True
#         elif (inp_c[0] == "vis_off"):
#             robot.showMode = False
#         elif (inp_c[0] == "read"):
#                 robot.read_points()
#         elif (inp_c[0] == "calib_axe"):
#                 robot.calibrate(str(inp_c[1]), '30')
#         elif (inp_c[0] == "opencv"):
#                 coord = robot.openCV(0, 5)
#                 logger.debug(f'coord in main = {coord}')
#                 #robot.move_all_xyz([coord[0], coord[1], 0])
#         elif (inp_c[0] == "cam"):
#                 robot.camera_calibrate(11)
#         elif (inp_c[0] == "camm"):
#                 robot.camera_calibrate_s(12)
#         elif (inp_c[0] == "cam2"):
#                 robot.camera_calibrate2()
#         elif (inp_c[0] == "take"):
#                 robot.take_object()
#         elif (inp_c[0] == "speed"):
#                 robot.position.speed = int(inp_c[1])
#         elif (inp_c[0] == "n"):
#                 robot.null_position()
#         elif (inp_c[0] == "enc"):
#                 robot.enc()
#         elif (inp_c[0] == "rot_t"):
#                 robot.move_theta(float(inp_c[1]))
#         elif (inp_c[0] == "crot"):
#                 robot.camera_calibrate_rot()
#         elif (inp_c[0] == '\n'):
#                 logger.debug("enter")
#         else:
#             print("Неправильная команда")
#   x = 0.23378473431949537 y = -0.28243977277327487 z = 0.12367249310609349
#   x = 0.23414067655672663 y = -0.28166264329105795 z = 0.12641368525064373
#   x = 0.23427166939727415 y = -0.2815486352975899 z = 0.12725524922403195
#   x = 0.2346889818155616 y = -0.28178645191304763 z = 0.12738980221272406

# перемещаю в рандомное положение

#   x = 0.24010852389258067 y = -0.28214600703892667 z = 0.14352120680188513
#   x = 0.23644844540959725 y = -0.2831234732060092 z = 0.1282371144417758
#   x = 0.23642186496407352 y = -0.28305583742444507 z = 0.12760506640496388

# перемещаю в рандомное положение

#   x = 0.23884239545920655 y = -0.2828691862998858 z = 0.13279068417955944
#   x = 0.23663102022558738 y = -0.28258806464632125 z = 0.12814576059646038

# x = 0.03699999999999994 y = 0.43500000000000005 z = 0.24000000000000005

# txmove 33 443 300 90 180 0 - точка в спектрофотометре
# txmove 33 443 200 90 180 0