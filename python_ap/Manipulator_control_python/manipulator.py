import time
import math
from math import (sin, cos, pi, atan2, sqrt, radians)
from scipy.spatial.transform import Rotation
import numpy as np
import serial
from loguru import logger
from config import DEFAULT_SETTINGS
from joint import Joint
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import cv2
import arucoOdometry
import threading
from controller import Controller

# from parse import parse


@dataclass(init=True)
class Position:
    """
    Class for keeping global position and orientation of manipulitor's wrist
    """
    x: float = 0  # mm
    y: float = 0  # mm
    z: float = 0  # mm
    theta: float = 0  # grad
    phi: float = 0  # grad
    psi: float = 0  # grad
    speed: int = 30

    def change(self, x, y, z, theta, phi, psi):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi
        self.psi = psi

    def change_speed(self, speed):
        self.speed = speed


class Manipulator:
    DH = {
        'a_1': 0.0642,
        'a_2': 0.305,
        'a_3': 0,
        'a_4': 0,
        'a_5': 0,
        'a_6': 0,
        'alpha_1': pi / 2,
        'alpha_2': 0,
        'alpha_3': pi / 2,
        'alpha_4': -pi / 2,
        'alpha_5': pi / 2,
        'alpha_6': 0,
        'd_1': 0.16977,
        'd_2': 0,
        'd_3': 0,
        'd_4': 0.22263,
        'd_5': 0,
        'd_6': 0.185,  # расстояние до конца схвата
        'displacement_theta_1': 0,
        'displacement_theta_2': 0,
        'displacement_theta_3': pi / 2,
        'displacement_theta_4': 0,
        'displacement_theta_5': 0,
        'displacement_theta_6': 0
    }

    dh_params = np.array([[DH['d_1'], DH['a_1'], DH['alpha_1'], DH['displacement_theta_1']],
                          [DH['d_2'], DH['a_2'], DH['alpha_2'], DH['displacement_theta_2']],
                          [DH['d_3'], DH['a_3'], DH['alpha_3'], DH['displacement_theta_3']],
                          [DH['d_4'], DH['a_4'], DH['alpha_4'], DH['displacement_theta_4']],
                          [DH['d_5'], DH['a_5'], DH['alpha_5'], DH['displacement_theta_5']],
                          [DH['d_6'], DH['a_6'], DH['alpha_6'], DH['displacement_theta_6']]
                          ])

    def __init__(self, teensy_port, arduino_port, baud, camera=False, controller_dualshock=False, checking_chanhing_of_angles=True, test_mode=False, continuouse_mesurement=False):
        self.read_config()
        self.test_mode = test_mode # данное поле можно включить, если нет подключения по сериал порту
        self.continuouse_mesurement = continuouse_mesurement
        self.controller_dualshock = controller_dualshock
        self.program_console = threading.Thread(target=self.startConsole, daemon=True)
        self.monitor = threading.Thread(target=self.monitorEnc, daemon=True)
        self.checking_chanhing_of_angles = checking_chanhing_of_angles
        self.console = True
        self.monitoringENC = False
        if camera:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if controller_dualshock:
            self.dualshock = Controller()
            self.dualshock_thread = threading.Thread(target=self.start_controller)
            self.start_thread_controller()
        self.last_inverse_pos = []  # хранит последние заданные координаты в обратную задачу кинематики
        self.logging = False  # включает вывод в консоль информацию
        self.showMode = False  # включает мод отображения в отдельном окне положения манипулятора
        self.robot = RobotSerial(self.dh_params)
        self.last_matrix = []  # содержит последнии матрицы преобразования координат
        self.last_dot_matrix = np.array([])  # содержит последнии перемноженные матрицы преобразования координат
        self.time_sleep = 3
        self.points = ""
        self.delta = 10
        self.WC = 'F'
        self.is_connected = False
        self.ACC_dur = 15
        self.ACC_spd = 10
        self.DEC_dur = 20
        self.DEC_spd = 5
        self.JogStepsStat = False
        self.joint_jog_degrees = 10
        self.joints = self.create_joints()
        self.calibration_direction = DEFAULT_SETTINGS['calibration_direction']
        self.motor_direction = DEFAULT_SETTINGS['motor_direction']
        self.position = Position()
        self.restore_position()
        self.calculate_direct_kinematics_problem()
        try:
            self.serial_teensy: serial.Serial = serial.Serial(teensy_port, baud)
            self.serial_arduino: serial.Serial = serial.Serial(arduino_port, baud)
            self.is_connected = True
        except serial.SerialException:
            logger.error("Serial port not defined")

    def start_program(self):
        self.program_console.start()
        self.monitor.start()
        self.program_console.join()
        self.monitor.join()
    def check_threads(self):
        if not self.program_console.is_alive():
            self.program_console.start()
        if not self.program_console.is_alive():
            self.program_console.start()
        if self.controller_dualshock:
            if not self.dualshock_thread.is_alive():
                self.dualshock_thread.start()
    def start_controller(self):
        button_rigth = np.array([0, 0])
        button_left = np.array([0, 0])

        button_upper = np.array([0, 0])
        button_lower = np.array([0, 0])

        button_up = np.array([0, 0])
        button_down = np.array([0, 0])

        grab = np.array([0, 0])
        absolve = np.array([0, 0])

        while True:
            self.dualshock.process_events()
            button_rigth[0] = self.dualshock.abs_state['HX']
            if button_rigth[0] == 1 and button_rigth[1] == 0:
                logger.debug("right")
            button_left[0] = self.dualshock.abs_state['HX']
            if button_left[0] == -1 and button_left[1] == 0:
                logger.debug("left")
            button_upper[0] = self.dualshock.abs_state['HY']
            if button_upper[0] == -1 and button_upper[1] == 0:
                logger.debug("upper")
            button_lower[0] = self.dualshock.abs_state['HY']
            if button_lower[0] == 1 and button_lower[1] == 0:
                logger.debug("lower")
            button_up[0] = self.dualshock.btn_state['L1']
            if button_up[0] == 1 and button_up[1] == 0:
                logger.debug("up")
            button_down[0] = self.dualshock.btn_state['R1']
            if button_down[0] == 1 and button_down[1] == 0:
                logger.debug("down")
            grab[0] = self.dualshock.btn_state['S']
            if grab[0] == 1 and grab[1] == 0:
                logger.debug("grab")
                self.grab()
            absolve[0] = self.dualshock.btn_state['N']
            if absolve[0] == 1 and absolve[1] == 0:
                logger.debug("absolve")
                self.absolve()

            button_rigth[1] = button_rigth[0]
            button_left[1] = button_left[0]

            button_upper[1] = button_upper[0]
            button_lower[1] = button_lower[0]

            button_up[1] = button_up[0]
            button_down[1] = button_down[0]

            grab[1] = grab[0]
            absolve[1] = absolve[0]
    def start_thread_controller(self):
        self.dualshock_thread.start()
        self.dualshock_thread.join()
    def startConsole(self):
        while self.console:
            try:
                inp = input("Введите команду \n")
                inp_c = inp.split()
                if (True):
                    if (inp == "exit"):
                        break
                    elif (inp == "c"):
                        self.auto_calibrate()
                    elif (inp == "conf"):
                        self.read_config()
                    elif (inp == "help"):
                        print("move_x [расстояние в мм] - передвижение по оси x в [мм]\n ")
                        print("move_y [расстояние в мм] - передвижение по оси y в [мм]\n ")
                        print("move_z [расстояние в мм] - передвижение по оси z в [мм]\n ")
                        print("calib - автокалибровка\n ")
                        self.info()
                    elif (inp_c[0] == "move_x"):
                        self.move_x(int(inp_c[1]))
                    elif (inp_c[0] == "check_th"):
                        self.check_threads()
                    elif (inp_c[0] == "move_y"):
                        self.move_y(int(inp_c[1]))
                    elif (inp_c[0] == "move_z"):
                        self.move_z(int(inp_c[1]))
                    elif (inp_c[0] == "servo"):
                        command = f"SV{0}P{inp_c[1]}\n"
                        print(command)
                        self.arduino_push(command)
                    elif (inp_c[0] == "g"):
                        self.grab()
                    elif (inp_c[0] == "a"):
                        self.absolve()
                    elif (inp_c[0] == "rot"):
                        self.jog_joint_c(int(inp_c[1]), int(inp_c[2]))
                    elif (inp_c[0] == "rots"):
                        self.jog_joint_s(int(inp_c[1]), int(inp_c[2]))
                    elif (inp_c[0] == "print"):
                        self.print()
                    elif (inp_c[0] == "move_all"):
                        self.jog_joints([inp_c[1], inp_c[2], inp_c[3], inp_c[4], inp_c[5], inp_c[6]])
                    elif (inp_c[0] == "add"):
                        points = f"inv,{self.last_inverse_pos[0]},{self.last_inverse_pos[1]},{self.last_inverse_pos[2]},{self.last_inverse_pos[3]},{self.last_inverse_pos[4]},{self.last_inverse_pos[5]}\n"
                        # logger.debug(self.points)
                        self.write_point(points)
                        # self.points += f"{self.joints[0].current_joint_angle},{self.joints[1].current_joint_angle},{self.joints[2].current_joint_angle},{self.joints[3].current_joint_angle},{self.joints[4].current_joint_angle},{self.joints[5].current_joint_angle}\n"
                    elif (inp_c[0] == "add2"):
                        points2 = f"dir,{self.joints[0].current_joint_angle},{self.joints[1].current_joint_angle},{self.joints[2].current_joint_angle},{self.joints[3].current_joint_angle},{self.joints[4].current_joint_angle},{self.joints[5].current_joint_angle}\n"
                        self.write_point(points2)
                    elif (inp_c[0] == "txmove"):
                        inv = self.calculate_inverse_kinematic_problem(
                            [float(inp_c[1]) / 1000, float(inp_c[2]) / 1000, float(inp_c[3]) / 1000,
                             np.radians(float(inp_c[4])),
                             np.radians(float(inp_c[5])), np.radians(float(inp_c[6]))])
                        logger.debug(inv)
                        ang = np.degrees(inv)
                        self.jog_joints([ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]])
                    elif (inp_c[0] == "vis"):
                        self.show()
                    elif (inp_c[0] == "vis_on"):
                        self.showMode = True
                    elif (inp_c[0] == "vis_off"):
                        self.showMode = False
                    elif (inp_c[0] == "read"):
                        self.read_points()
                    elif (inp_c[0] == "calib_axe"):
                        self.calibrate(str(inp_c[1]), '30')
                    elif (inp_c[0] == "opencv"):
                        coord = self.openCV(0, 5)
                        logger.debug(f'coord in main = {coord}')
                        # self.move_all_xyz([coord[0], coord[1], 0])
                    elif (inp_c[0] == "cam"):
                        self.camera_calibrate(17)
                    elif (inp_c[0] == "camm"):
                        self.camera_calibrate_s(12)
                    elif (inp_c[0] == "cam2"):
                        self.camera_calibrate2()
                    elif (inp_c[0] == "take"):
                        self.take_object()
                    elif (inp_c[0] == "speed"):
                        self.position.speed = int(inp_c[1])
                    elif (inp_c[0] == "n"):
                        self.null_position()
                    elif (inp_c[0] == "enc"):
                        self.enc()
                    elif (inp_c[0] == "rot_t"):
                        self.move_theta(float(inp_c[1]))
                    elif (inp_c[0] == "crot"):
                        self.camera_calibrate_rot()
                    elif (inp_c[0] == '\n'):
                        logger.debug("enter")
                    elif (inp_c[0] == 'jog'):
                        self.jog_joints_test([inp_c[1], inp_c[2], inp_c[3], inp_c[4], inp_c[5], inp_c[6]])
                    elif (inp_c[0] == 'calc_angle'):
                        self.calc_angle(float(inp_c[1]), self.joints[int(inp_c[2])])
                    elif (inp_c[0] == 'calc_step'):
                        self.calc_steps(float(inp_c[1]), self.joints[int(inp_c[2])])
                    elif (inp_c[0] == 'check'):
                        self.getRobotPosition()
                    elif (inp_c[0] == 'rotate_relative'):
                        self.rotate_relative([inp_c[1], inp_c[2], inp_c[3], inp_c[4], inp_c[5], inp_c[6]])
                    elif (inp_c[0] == 'rot_joint'): # угол и номер джойнта
                        self.joints_rot(inp_c[1], inp_c[2])
                    else:
                        print("Неправильная команда")
            except:
                logger.error("произошла ошибка")
    def monitorEnc(self):
        angles = np.round(np.array([self.joints[0].current_joint_step, self.joints[1].current_joint_step,
                            self.joints[2].current_joint_step, self.joints[3].current_joint_step,
                            self.joints[4].current_joint_step, self.joints[5].current_joint_step]))
        if self.continuouse_mesurement:
            while True:
                if self.monitoringENC:
                    try:
                        self.getRobotPosition()
                    except:
                        logger.debug("Упал поток")
                        self.check_threads()
                    self.conversion_steps_angles()
                if self.checking_chanhing_of_angles:
                    str_angles = f"{self.joints[0].current_joint_angle}, {self.joints[1].current_joint_angle}, " + \
                    f"{self.joints[2].current_joint_angle}, {self.joints[3].current_joint_angle}, " + \
                    f"{self.joints[4].current_joint_angle}, {self.joints[5].current_joint_angle}"
                    new_angles = np.round(np.array([self.joints[0].current_joint_step, self.joints[1].current_joint_step,
                                self.joints[2].current_joint_step, self.joints[3].current_joint_step,
                                self.joints[4].current_joint_step, self.joints[5].current_joint_step]))
                    for i in range(6):
                        if abs(new_angles[i] - angles[i]) > 3/self.joints[i].degrees_per_step:
                            logger.debug(f"The angles are changing: \n {str_angles}")
                            #logger.debug(new_angles.all())

                angles = new_angles
                time.sleep(0.1)
        else:
            logger.debug(f"continuouse_mesurement = {self.continuouse_mesurement}")

    def conversion_steps_angles(self):
        config_converse = [1, 0, 0, 1]
        for i in range(6):
            if self.joints[i].positive_angle_limit > self.joints[i].negative_angle_limit:
                delta = self.joints[i].positive_angle_limit
                if i == 2:
                    inv = -1
                else:
                    inv = 1
                self.joints[i].current_joint_angle = (-abs(self.joints[i].current_joint_step)
                                                      * self.joints[i].degrees_per_step + delta) * self.joints[
                                                         i].motor_dir*inv
            else:
                if i == 2:
                    inv = -1
                else:
                    inv = 1
                delta = self.joints[i].negative_angle_limit
                self.joints[i].current_joint_angle = -(-abs(self.joints[i].current_joint_step)
                                                      * self.joints[i].degrees_per_step + delta)*inv

    def getRobotPosition(self):
        commandCalc = "GP" + "U" + str(self.joints[0].current_joint_step) + "V" + str(
            self.joints[1].current_joint_step) + "W" + str(self.joints[2].current_joint_step) + "X" + str(
            self.joints[3].current_joint_step) + "Y" + str(self.joints[4].current_joint_step) + "Z" + str(
            self.joints[5].current_joint_step) + "\n"
        self.serial_teensy.write(commandCalc.encode())
        RobotCode = str(self.serial_teensy.readline())
        RobotCode = RobotCode.replace('Done', '')
        A = RobotCode.find('A')
        B = RobotCode.find('B')
        C = RobotCode.find('C')
        D = RobotCode.find('D')
        E = RobotCode.find('E')
        F = RobotCode.find('F')
        end = RobotCode.find('r')
        if A != -1 and not RobotCode[A + 1: B] == '':
            Asteps = int(RobotCode[A + 1: B])
        else:
            Asteps = self.joints[0].current_joint_step

        if B != -1 and not RobotCode[B + 1: C] == '':
            Bsteps = int(RobotCode[B + 1: C])
        else:
            Bsteps = self.joints[1].current_joint_step
        # Csteps = int(RobotCode[C + 1: D])
        if C != -1 and not RobotCode[C + 1: D] == '':
            Csteps = int(RobotCode[C + 1: D])
        else:
            Csteps = self.joints[2].current_joint_step
        if D != -1 and not RobotCode[D + 1: E] == '':
            Dsteps = int(RobotCode[D + 1: E])
        else:
            Dsteps = self.joints[3].current_joint_step

        if E != -1 and not RobotCode[E + 1: F] == '':
            Esteps = int(RobotCode[E + 1: F])
        else:
            Esteps = self.joints[4].current_joint_step

        if F != -1 and not RobotCode[F + 1: end-1] == '':
            Fsteps = int(RobotCode[F + 1: end-1])
        else:
            Fsteps = self.joints[5].current_joint_step
        steps = np.array([Asteps, Bsteps, Csteps, Dsteps, Esteps, Fsteps])
        for i in range(6):
            if not i == 4:
                continue
            self.joints[i].current_joint_step = steps[i]

    def show_workspace(self):
        self.robot.ws_division = 6
        self.robot.show(ws=True)

    def read_config(self):
        file = open("config", "r")
        config_text = file.read()
        config_joints = config_text.split("\n")
        while '' in config_joints: config_joints.remove('')
        massive_val = []
        for el in config_joints:
            name_val = el.split(' = ')
            massive_val.append(name_val)
        for com in massive_val:
            DEFAULT_SETTINGS[com[0]] = com[1]

    def save_position(self):
        if (self.logging == True):
            logger.debug("Запись в файл")
        file = open("lastPos", "w")
        file.truncate()
        file.write(
            f"{self.joints[0].current_joint_angle}, {self.joints[1].current_joint_angle}, "
            f"{self.joints[2].current_joint_angle}, {self.joints[3].current_joint_angle}, "
            f"{self.joints[4].current_joint_angle}, {self.joints[5].current_joint_angle}")
        file.close()
        file = open("lastInversePos", "w")
        file.truncate()
        file.write(
            f"{self.last_inverse_pos[0]}, {self.last_inverse_pos[1]}, "
            f"{self.last_inverse_pos[2]}, {self.last_inverse_pos[3]}, "
            f"{self.last_inverse_pos[4]}, {self.last_inverse_pos[5]}")
        file.close()

    def restore_position(self):
        file = open("lastPos", "r")
        text = file.read()
        angles = text.split(",")
        for i, angle in enumerate(angles):
            angle = float(angle)
            self.joints[i].current_joint_angle = angle
        file = open("lastInversePos", "r")
        text = file.read()
        angles = text.split(",")
        massive = []
        for i, xyzabc in enumerate(angles):
            xyzabc = float(xyzabc)
            massive.append(xyzabc)
        self.last_inverse_pos = np.array(massive)

    def finish(self):
        self.serial_teensy.close()
        self.serial_arduino.close()

    def write_point(self, string):
        file = open("points.txt", "a")
        file.write(string)
        file.close()

    def read_points(self):
        file = open("points.txt", "r")
        commands = file.read()
        command = commands.split("\n")
        for i in range(len(command)):
            # if (command[i] == "grab"):
            angles = command[i].split(",")
            # mas = []
            if (angles[0] == "inv"):
                x = float(angles[1])
                y = float(angles[2])
                z = float(angles[3])
                a = float(angles[4])
                b = float(angles[5])
                g = float(angles[6])
                logger.debug([x, y, z, a, b, g])
                xyzabc = self.calculate_inverse_kinematic_problem([x, y, z, a, b, g])
                logger.debug(xyzabc)
                ang = np.degrees(xyzabc)
                logger.debug(ang)
                self.jog_joints([ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]])
            if (angles[0] == "move_z"):
                self.move_z(float(angles[1]) * 1000)
            if (angles[0] == "move_x"):
                self.move_x(float(angles[1]) * 1000)
            if (angles[0] == "move_y"):
                self.move_y(float(angles[1]) * 1000)
            if (angles[0] == "move_xyz"):
                self.move_xyz([self.position.x + float(angles[1]), self.position.y + float(angles[2]),
                               self.position.z + float(angles[3])])
            if (angles[0] == "rest"):
                self.null_position()
            if (angles[0] == "cam"):
                self.camera_calibrate(int(angles[1]))
            if (angles[0] == "camr"):
                self.camera_calibrate_s(int(angles[1]), int(angles[2]), count=int(angles[3]), state=int(angles[4]),
                                        delta=[angles[5], angles[6], 0])
            if (angles[0] == "take"):
                self.take_object()
            if (angles[0] == "dir"):
                theta = [float(angles[1]), float(angles[2]), float(angles[3]), float(angles[4]),
                         float(angles[5]), float(angles[6])]
                theta = np.degrees(theta)
                self.jog_joints(theta)
            if (angles[0] == "grab"):
                self.grab()
            if (angles[0] == "abs"):
                self.absolve()
            if (angles[0] == "calib"):
                self.auto_calibrate()
            if (angles[0] == "sleep"):
                logger.debug(f"sleep {angles[1]}------------------------------------->")
                time.sleep(float(angles[1]))
            if (angles[0] == "speed"):
                logger.debug(f"speed = {angles[1]}------------------------------------->")
                self.position.speed = int(angles[1])

    def get_joints_angles(self) -> list:
        return [joint.current_joint_angle for joint in self.joints]

    def move_to(self, command):
        self.serial_teensy.write(command.encode())

    def info(self):
        for i in range(6):
            print(f"Угол концевого переключателя {i + 1}го звена: {self.joints[i].endstop_angle} \n"
                  f"Предельный угол {i + 1}го звена: {self.joints[i].angle_limit} \n"
                  f"\n")

    def check_angle(self, angle, joint: Joint):
        if joint.endstop_angle > joint.angle_limit:
            if angle > joint.endstop_angle or angle < joint.angle_limit:
                return False
            else:
                return True
        elif joint.endstop_angle < joint.angle_limit:
            if angle < joint.endstop_angle or angle > joint.angle_limit:
                return False
            else:
                return True
        else:
            return False

    def calc_angle(self, angle, joint: Joint):
        angle = float(angle)
        # if (joint.positive_angle_limit > 0):
        #     if (angle > joint.positive_angle_limit or angle < joint.negative_angle_limit):
        #         logger.error(f"Угол звена {joint.get_name_joint()} превышает лимит")
        #         return [0, 0, True]
        # if (joint.positive_angle_limit < 0):
        #     if (angle < joint.positive_angle_limit or angle > joint.negative_angle_limit):
        #         logger.error(f"Угол звена {joint.get_name_joint()} превышает лимит")
        #         return [0, 0, True]
        error = self.check_angle(angle, joint)
        # Расчет направления двигателей
        x = joint.current_joint_angle  # joint.current_joint_step*joint.degrees_per_step + joint.negative_angle_limit
        arc = abs(angle - x)
        if joint.motor_dir == 1:
            if angle < x:
                drive_direction = 1
            elif angle > x:
                drive_direction = 0
            else:
                drive_direction = 0
        elif joint.motor_dir == -1:
            if angle > x:
                drive_direction = 1
            elif angle < x:
                drive_direction = 0
            else:
                drive_direction = 0
        return [arc, drive_direction, error]

    def inverse_one_zero(self, one_or_zero):
        if one_or_zero == 0:
            one_or_zero = 1
        elif one_or_zero == 1:
            one_or_zero = 0
        else:
            return None
        return one_or_zero



    def calc_steps(self, steps, joint: Joint):
        steps = int(steps)
        # if (joint.positive_angle_limit > 0):
        #     if (angle > joint.positive_angle_limit or angle < joint.negative_angle_limit):
        #         logger.error(f"Угол звена {joint.get_name_joint()} превышает лимит")
        #         return [0, 0, True]
        # if (joint.positive_angle_limit < 0):
        #     if (angle < joint.positive_angle_limit or angle > joint.negative_angle_limit):
        #         logger.error(f"Угол звена {joint.get_name_joint()} превышает лимит")
        #         return [0, 0, True]
        # Расчет направления двигателей
        #delta = joint.negative_angle_limit / joint.degrees_per_step
        x = joint.current_joint_step  # x - текущая точка
        logger.debug(x)
        arc = abs(steps - x)
        logger.debug(arc)
        if joint.motor_dir == 1:
            if steps > x:
                drive_direction = 0
            if steps < x:
                drive_direction = 1
            if steps == x:
                logger.error(f"Звено {joint.get_name_joint()} уже в этом положении")
                drive_direction = 1
                arc = 0
        if joint.motor_dir == -1:
            if steps > x:
                drive_direction = 1
            if steps < x:
                drive_direction = 0
            if steps == x:
                logger.error(f"Звено {joint.get_name_joint()} уже в этом положении")
                drive_direction = 1
                arc = 0
        error = False
        logger.debug(f"arc, drive, error - {[arc, drive_direction, error]}")
        return [arc, drive_direction, error]  # дуга [шаг], направление (1 или 0), ошибка True or False




    def jog_joint_c(self, number_of_joint, degrees):
        angles = [self.joints[0].current_joint_angle, self.joints[1].current_joint_angle,
                  self.joints[2].current_joint_angle, self.joints[3].current_joint_angle,
                  self.joints[4].current_joint_angle, self.joints[5].current_joint_angle]
        angles[number_of_joint] = degrees
        self.jog_joints(angles)
    def jog_joint_s(self, number_of_joint, degrees):
        degrees_at_steps = degrees#degrees/self.joints[number_of_joint].degrees_per_step  # град/(град/шаг)
        steps = [self.joints[0].current_joint_step, self.joints[1].current_joint_step,
                 self.joints[2].current_joint_step, self.joints[3].current_joint_step,
                 self.joints[4].current_joint_step, self.joints[5].current_joint_step]
        steps[number_of_joint] = degrees_at_steps
        self.jog_joints_steps(steps)
    def jog_joints(self, degrees):
        degrees = [float(x) for x in degrees]
        joint_commands = []
        errors = []
        angles = []
        for i in range(6):
            d = self.calc_angle(degrees[i], self.joints[i])
            arc = d[0]
            direction = d[1]
            j_jog_steps = abs(int(arc / self.joints[i].degrees_per_step))
            if (DEFAULT_SETTINGS['motor_inv'][i] == '1'):
                direction = self.inverse_one_zero(direction)
            joint_commands.append(f"{self.joints[i].get_name_joint()}{direction}{j_jog_steps}")
            errors.append(d[2])
            angles.append(degrees[i])
        if (not errors[0] and not errors[1] and not errors[2] and not errors[3] and not errors[4] and not errors[5]):
            for i in range(6):
                if self.logging:
                    logger.debug(f"Changing angle {self.joints[i].current_joint_angle} - > {angles[i]}")
                self.joints[i].current_joint_angle = angles[i]
            command = f"MJ{''.join(joint_commands)}S{self.position.speed}G{15}H{10}I{20}K{5}\n"
            self.teensy_push(command)
            self.save_position()
            self.calculate_direct_kinematics_problem()
            if (self.logging == True):
                logger.debug(f"Запись углов в джойнты: {angles}")
        else:
            logger.error("Команда не отправилась, превышен лимит одного из джойнтов")

    def jog_joints_test(self, degrees, steps=False):
        degrees = [float(x) for x in degrees]
        joint_commands = []
        errors = []
        angles = []
        for joint in self.joints:
            logger.debug(f'current_steps 1 = {joint.current_joint_step}')
        self.getRobotPosition()
        time.sleep(0.5)
        degrees_in_steps = []
        current_steps = []
        for index, joint in enumerate(self.joints):  # Перевод входного угла в шаги
            current_steps.append(joint.current_joint_step)
            if joint.positive_angle_limit > joint.negative_angle_limit:
                delta_angle = joint.negative_angle_limit
            else:
                delta_angle = joint.positive_angle_limit
            degrees_in_steps.append((degrees[index] + abs(delta_angle))/joint.degrees_per_step)
        arc_m = []
        direction_m = []
        for i in range(6):
            d = self.calc_angle(degrees[i], self.joints[i])
            arc = d[0]
            direction = d[1]

            direction_m.append(direction)
            j_jog_steps = abs(int(arc / self.joints[i].degrees_per_step))
            arc_m.append(j_jog_steps)
            joint_commands.append(f"{self.joints[i].get_name_joint()}{direction}{j_jog_steps}")
            errors.append(d[2])
            angles.append(degrees[i])
        if (not errors[0] and not errors[1] and not errors[2] and not errors[3] and not errors[4] and not errors[5]):
            for i in range(6):
                self.joints[i].current_joint_angle = angles[i]
            command = f"MJ{''.join(joint_commands)}S{self.position.speed}G{15}H{10}I{20}K{5}\n"
            self.teensy_push(command)
            time.sleep(5)
            self.getRobotPosition()
            time.sleep(5)
            delta = self.check_angle(current_steps, arc_m)
            self.rotate_relative(delta) #=---------------------
            self.save_position()
            self.calculate_direct_kinematics_problem()
            if (self.logging == True):
                logger.debug(f"Запись углов в джойнты: {angles}")
        else:
            logger.error("Команда не отправилась, превышен лимит одного из джойнтов")

    # def check_angle(self, steps, arc_in_steps):
    #     current_steps = []
    #     for joint in self.joints:
    #         current_steps.append(joint.current_joint_step)
    #     steps[2] -= self.joints[2].step_limit
    #     steps[5] -= self.joints[5].step_limit
    #     steps_ = np.array(current_steps) - np.array(steps)
    #     steps_tilda = arc_in_steps
    #     error = steps_ - steps_tilda
    #     return error
    def jog_joints_steps(self, steps, degrees=False):
        if not degrees:
            steps = [int(x) for x in steps]
        if degrees:
            steps = [float(x) for x in steps]
        joint_commands = []
        errors = []
        steps_massive = []

        for i in range(6):
            if degrees:
                if self.joints[i].positive_angle_limit > self.joints[i].negative_angle_limit:
                    delta = self.joints[i].positive_angle_limit / self.joints[i].degrees_per_step
                else:
                    delta = self.joints[i].negative_angle_limit / self.joints[i].degrees_per_step
                steps[i] = steps[i]/self.joints[i].degrees_per_step + delta
            d = self.calc_steps(steps[i], self.joints[i])
            arc = d[0]
            # logger.debug(self.joints[i].motor_dir)
            direction = d[1]
            j_jog_steps = abs(int(arc))  #  abs(int(arc / self.joints[i].degrees_per_step))
            joint_commands.append(f"{self.joints[i].get_name_joint()}{direction}{j_jog_steps}")
            errors.append(d[2])
            # if (d[2] != True):
            #     logger.debug(f"Запись в джойнт {i+1}")
            steps_massive.append(steps[i])
            # self.joints[i].current_joint_angle = degrees[i]
        if (not errors[0] and not errors[1] and not errors[2] and not errors[3] and not errors[4] and not errors[5]):
        #     for i in range(6):
        #         self.joints[i].current_joint_angle = angles[i]
            command = f"MJ{''.join(joint_commands)}S{self.position.speed}G{15}H{10}I{20}K{5}\n"
            self.teensy_push(command)
            self.save_position()
            self.calculate_direct_kinematics_problem()
            if self.logging:
                logger.debug(f"Запись шагов в джойнты: {steps}")
        else:
            logger.error("Команда не отправилась, превышен лимит одного из джойнтов")

    def joints_rot(self, degree, number_of_joint):  # поворачивает джойнт на угол от текущего, degree - угол в градусах,
        # number_of_joint - номер джойнта, начиная с 0
        degree = float(degree)
        steps = np.array([0, 0, 0, 0, 0, 0])
        dir = np.array([0, 0, 0, 0, 0, 0])
        j_jog_steps = abs(int(degree / self.joints[number_of_joint].degrees_per_step))
        steps[number_of_joint] = j_jog_steps
        if degree > 0.0:
            direction = self.joints[number_of_joint].motor_direction
        elif degree < 0.0:
            direction = self.inverse_one_zero(self.joints[number_of_joint].motor_direction)
        else:
            direction = 0
        dir[number_of_joint] = direction
        # logger.debug(direction)
        command = f"MJA{dir[0]}{steps[0]}B{dir[0]}{steps[1]}C{dir[0]}{steps[2]}D{dir[0]}{steps[3]}E{dir[0]}{steps[4]}F{dir[0]}{steps[5]}S{self.position.speed}G{15}H{10}I{20}K{5}\n"
        self.teensy_push(command)
        self.joints[number_of_joint].current_joint_angle = self.joints[number_of_joint].current_joint_angle + degree
        self.save_position()
        self.calculate_direct_kinematics_problem()
        # позже необходимо ввести проверку предельного угла

    def rotate_relative(self, steps):
        inverse_massive = np.array([-1, -1, -1, 1, -1, 1])
        inverse_massive = np.array([-1, -1, 0, 1, -1, 0])
        steps = [int(x) for x in steps]
        steps = np.array(steps)*inverse_massive

        directions = []
        for step in steps:
            if step > 0:
                directions.append(1)
            if step < 0:
                directions.append(0)
            else:
                directions.append(0)
        joint_commands = []
        for i, joint in enumerate(self.joints):
            joint_commands.append(f"{self.joints[i].get_name_joint()}{directions[i]}{abs(steps[i])}")
        command = f"MJ{''.join(joint_commands)}S{self.position.speed}G{15}H{10}I{20}K{5}\n"
        self.teensy_push(command)
    def apply_robot_calibration(self, robot_code: str):
        faults = [
            robot_code[4],
            robot_code[5],
            robot_code[6],
            robot_code[7],
            robot_code[8],
            robot_code[9]
        ]
        joint_name_indexes = [
            robot_code.find('A'),
            robot_code.find('B'),
            robot_code.find('C'),
            robot_code.find('D'),
            robot_code.find('E'),
            robot_code.find('F')
        ]

        for i, joint in enumerate(self.joints):
            if not joint.open_loop_stat and faults[i] == '1':
                logger.error(
                    f'{joint.get_name_joint()} COLLISION OR OUT OF CALIBRATION')

                if i < 5:
                    joint.current_joint_step = int(robot_code[joint_name_indexes[i] + 1: joint_name_indexes[i + 1]])
                else:
                    joint.current_joint_step = int(robot_code[joint_name_indexes[i] + 1:])

                joint.current_joint_angle = round(joint.negative_angle_limit + (joint.current_joint_step *
                                                                                joint.degrees_per_step), 2)
                # self.stop_program()
        self.calculate_direct_kinematics_problem()
        # self.save_data()

    def teensy_push(self, command):
        if self.logging:
            logger.debug(f'Teensy push {command}')
        if not self.test_mode:
            self.serial_teensy.write(command.encode())

    def arduino_push(self, command):
        if self.logging:
            logger.debug(f'Arduino push: {command}')
       # if not self.test_mode:
        self.serial_arduino.write(command.encode())

    @staticmethod
    def create_joints():
        joints_name = ['A', 'B', 'C', 'D', 'E', 'F']
        joints = [Joint(i + 1,
                        float(DEFAULT_SETTINGS[f'J{i + 1}_endstop_angle']) + float(DEFAULT_SETTINGS[f'J{i + 1}_delta']),
                        float(DEFAULT_SETTINGS[f'J{i + 1}_angle_limit']) + float(DEFAULT_SETTINGS[f'J{i + 1}_delta']),
                        DEFAULT_SETTINGS[f'J{i + 1}_step_limit'])
                  for i in range(6)]
        #logger.debug(joints[0].endstop_angle)
        for joint, joint_name in zip(joints, joints_name):
            joint.set_name_joint(joint_name)

        for i, joint in enumerate(joints):
            joint.motor_dir = int(DEFAULT_SETTINGS[f'J{i + 1}_rot_dir'])
            joint.degrees_per_step = float(DEFAULT_SETTINGS[f'J{i + 1}_per_step'])

        return joints

    def get_calibration_drive(self):
        calibration_drive = []
        for cd, md in zip(self.calibration_direction, self.motor_direction):
            if cd == md:
                calibration_drive.append('0')
            else:
                calibration_drive.append('1')
        return calibration_drive

    def get_calibration_drive_auto(self):
        calibration_drive = []
        for cd, md in zip(self.calibration_direction, self.motor_direction):
            if cd == md:
                calibration_drive.append('1')
            else:
                calibration_drive.append('0')
        return calibration_drive

    def calibrate(self, calibration_axes: str, speed: str):
        # val = False
        # if self.monitoringENC:
        #     self.monitoringENC = False
        #     val = True
        axes = [axis for axis in calibration_axes]

        steps = []
        for i, axis in enumerate(axes):
            if axis == '1':
                steps.append(self.joints[i].step_limit)
            else:
                steps.append(0)

        calibration_drive = self.get_calibration_drive()
        #logger.debug(calibration_drive)
        joint_calibration_drive_and_step = [f"{joint.get_name_joint()}{cd}{step}"
                                            for joint, cd, step in zip(self.joints, calibration_drive, steps)]
        # for i in range(6):
        #     logger.debug(self.joints[i].step_limit)
        command = f"LL{''.join(joint_calibration_drive_and_step)}S{speed}\n"
        self.teensy_push(command)
        if self.logging:
            logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()
        calibration_value = self.serial_teensy.read()
        #if calibration_value == b'P':
        if True:
            # calibration_status = 1
            for joint, cd, axis in zip(self.joints, self.calibration_direction, axes):
                if axis == '1':
                    # print
                    joint.current_joint_step = 0
                    if joint.name_joint == 'C' or joint.name_joint == 'F':
                        joint.current_joint_step = joint.step_limit
                        # logger.debug("polka-------------------------")
                    joint.current_joint_angle = joint.endstop_angle
                    # logger.debug("calib")
                    # logger.debug(joint.endstop_angle)
                    # if cd == '0':
                    #     if (joint.motor_dir == 1):
                    #         # joint.current_joint_step = 0
                    #         joint.current_joint_angle = joint.endstop_angle
                    #     else:
                    #         # joint.current_joint_step = joint.step_limit
                    #         joint.current_joint_angle = joint.angle_limit
                    # else:
                    #     if (joint.motor_dir == -1):
                    #         # joint.current_joint_step = joint.step_limit
                    #         joint.current_joint_angle = joint.negative_angle_lim
                    #     else:
                    #         # joint.current_joint_step = 0
                    #         joint.current_joint_angle = joint.positive_angle_limit

            logger.success('CALIBRATION SUCCESSFUL')
        elif calibration_value == b'F':
            # calibration_status = 0
            logger.error('CALIBRATION FAILED')
        else:
            logger.warning('NO CAL FEEDBACK FROM ARDUINO')

        self.calculate_direct_kinematics_problem()
        # self.save_data()
        joints_current_steps = [f"{joint.get_name_joint()}{joint.current_joint_step}" for joint in self.joints]
        command = f'LM{"".join(joints_current_steps)}\n'
        self.teensy_push(command)
        if self.logging:
            logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()
        # if val:
        #     self.monitoringENC = True

    def auto_calibrate(self):
        self.monitoringENC = False
        # self.calibrate('100010', '40')
        # self.calibrate('011101', '40')
        self.calibrate('111111', '40')
        cd = self.get_calibration_drive_auto()  # направление калибровки
        command = f"MJA{cd[0]}500B{cd[1]}500C{cd[2]}500D{cd[3]}500E{cd[4]}500F{cd[5]}0" \
                  f"S15G10H10I10K10\n"
        self.teensy_push(command)
        if self.logging:
            logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()
        time.sleep(2.5)
        self.calibrate('111111', '8')
        self.calculate_direct_kinematics_problem()

    def null_position(self):
        angles = [0, 90, -90, 0, -90, 0]
        self.jog_joints(angles)
        self.calculate_direct_kinematics_problem()

    def calculate_direct_kinematics_problem(self):
        for joint in self.joints:
            if joint.get_current_joint_angle() == 0:
                joint.current_joint_angle = 0.0000000001
        theta = np.array([np.radians(self.joints[0].current_joint_angle),
                          np.radians(self.joints[1].current_joint_angle),
                          np.radians(self.joints[2].current_joint_angle),
                          np.radians(self.joints[3].current_joint_angle),
                          np.radians(self.joints[4].current_joint_angle),
                          np.radians(self.joints[5].current_joint_angle)])
        robot = RobotSerial(self.dh_params)
        # robot.ws_lim = self.limits
        f = robot.forward(theta)
        if (self.logging == True):
            logger.debug(f"xyz = {np.round(f.t_3_1.reshape([3, ]), 4)}, abc = {np.round(np.degrees(f.euler_3), 4)}")
        x = f.t_3_1.reshape([3, ])[0]
        y = f.t_3_1.reshape([3, ])[1]
        z = f.t_3_1.reshape([3, ])[2]
        theta = f.euler_3[0]
        phi = f.euler_3[1]
        psi = f.euler_3[2]
        self.position.change(x, y, z, theta, phi, psi)
        if (self.showMode):
            robot.show()
        return np.hstack([f.t_3_1.reshape([3, ]), f.euler_3])

    def calculate_direct2(self):
        cja = [float(self.joints[0].current_joint_angle), float(self.joints[1].current_joint_angle),
               float(self.joints[2].current_joint_angle),
               float(self.joints[3].current_joint_angle), float(self.joints[4].current_joint_angle),
               float(self.joints[5].current_joint_angle)]
        cja = [0.0, 0, 0, 0.0, 0, 0]
        cja = list(map(radians, cja))

        T = []
        for i in range(6):
            d = self.DH[f'displacement_theta_{i + 1}']
            T.append(np.array(
                [[cos(cja[i] + d), -sin(cja[i] + d) * cos(self.DH[f'alpha_{i + 1}']),
                  sin(cja[i] + d) * sin(self.DH[f'alpha_{i + 1}']),
                  self.DH[f'a_{i + 1}'] * cos(cja[i] + d)],
                 [sin(cja[i] + d), cos(cja[i] + d) * cos(self.DH[f'alpha_{i + 1}']),
                  -cos(cja[i] + d) * sin(self.DH[f'alpha_{i + 1}']),
                  self.DH[f'a_{i + 1}'] * sin(cja[i] + d)],
                 [0, sin(self.DH[f'alpha_{i + 1}']), cos(self.DH[f'alpha_{i + 1}']), self.DH[f'd_{i + 1}']],
                 [0, 0, 0, 1]]))
        self.last_matrix = T
        return T

    def matrix_dot_all(self, array_matrix):
        T0_6 = ((((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(
            array_matrix[4])).dot(array_matrix[5])
        self.last_dot_matrix = T0_6
        return T0_6

    def matrix_create(self):
        cja = [float(self.joints[0].current_joint_angle), float(self.joints[1].current_joint_angle),
               float(self.joints[2].current_joint_angle),
               float(self.joints[3].current_joint_angle), float(self.joints[4].current_joint_angle),
               float(self.joints[5].current_joint_angle)]
        cja = list(map(radians, cja))
        T = []
        for i in range(6):
            d = self.DH[f'displacement_theta_{i + 1}']
            T.append(np.array(
                [[cos(cja[i] + d), -sin(cja[i] + d) * cos(self.DH[f'alpha_{i + 1}']),
                  sin(cja[i] + d) * sin(self.DH[f'alpha_{i + 1}']),
                  self.DH[f'a_{i + 1}'] * cos(cja[i] + d)],
                 [sin(cja[i] + d), cos(cja[i] + d) * cos(self.DH[f'alpha_{i + 1}']),
                  -cos(cja[i] + d) * sin(self.DH[f'alpha_{i + 1}']),
                  self.DH[f'a_{i + 1}'] * sin(cja[i] + d)],
                 [0, sin(self.DH[f'alpha_{i + 1}']), cos(self.DH[f'alpha_{i + 1}']), self.DH[f'd_{i + 1}']],
                 [0, 0, 0, 1]]))
        self.last_matrix = T
        return T

    def matrix_dot_all(self, array_matrix):
        T0_6 = ((((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(
            array_matrix[4])).dot(array_matrix[5])

        return T0_6

    def matrix_dot(self, array_matrix, num1, num2):
        # global matrix
        matrix = None
        if num1 == 0:  # T1 * T{num2}, то есть, если num1 = 0, а num2 = 1, то T1*T2 or num2 = 5, то T1*T6
            if num2 == 1:  # T0_1 = T1
                matrix = array_matrix[0]
            if num2 == 2:  # T0_2 = T1*T2
                matrix = array_matrix[0].dot(array_matrix[1])
            if num2 == 3:  # T0_3 = T1*T2*T3
                matrix = (array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])
            if num2 == 4:  # T0_4 = T1*T2*T3*T4
                matrix = ((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])
            if num2 == 5:  # T0_5 = T1*T2*T3*T4*T5
                matrix = (((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(
                    array_matrix[4])
            if num2 == 6:  # T0_6 = T1*T2*T3*T4*T5*T6
                matrix = ((((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(
                    array_matrix[4])).dot(array_matrix[5])
        elif num1 == 1:
            if num2 == 2:  # T1_2 =  T2
                matrix = array_matrix[1]
            if num2 == 3:  # T1_3 = T2*T3
                matrix = array_matrix[1].dot(array_matrix[2])
            if num2 == 4:  # T1_4 = T2*T3*T4
                matrix = (array_matrix[1].dot(array_matrix[2])).dot(array_matrix[3])
            if num2 == 5:  # T1_5 = T2*T3*T4*T5
                matrix = ((array_matrix[1].dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])
        elif num1 == 2:
            if num2 == 3:  # T2_3 = T3
                matrix = array_matrix[2]
            if num2 == 4:  # T2_4 = T3*T4
                matrix = array_matrix[2].dot(array_matrix[3])
            if num2 == 5:  # T2_5 = T3*T4*T5
                matrix = (array_matrix[2].dot(array_matrix[3])).dot(array_matrix[4])
            if num2 == 6:  # T2_6 = T3*T4*T5*T6
                matrix = ((array_matrix[2].dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])
        elif num1 == 3:
            if num2 == 4:  # T3_4 = T4
                matrix = array_matrix[3]
            if num2 == 5:  # T3_5 = T4*T5
                matrix = array_matrix[3].dot(array_matrix[4])
            if num2 == 6:  # T3_6 = T4*T5*T6
                matrix = (array_matrix[3].dot(array_matrix[4])).dot(array_matrix[5])
        elif num1 == 4:
            if num2 == 5:  # T4_5 = T5
                matrix = array_matrix[4]
            if num2 == 6:  # T4_6 = T5*T6
                matrix = array_matrix[4].dot(array_matrix[5])
        elif num1 == 5:
            if num2 == 6:  # T5_6 = T6
                matrix = array_matrix[5]
        return matrix

    def angular_Euler_calculation(self, transform_matrix0_6):
        # # global theta, fi, psi
        rot = transform_matrix0_6[0:3, 0:3]
        r = Rotation.from_matrix(rot)
        quat = r.as_quat()
        angles = self.euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
        theta = round(angles[0], 4)
        fi = round(angles[1], 4)
        psi = round(angles[2], 4)
        return [theta, fi, psi]  # углы Эйлера схвата в главной системе координат,  fi -z,

    def calculate_inverse_kinematic_problem(self, x_y_z_phi_theta_psi, theta3plus=False):
        self.last_inverse_pos = x_y_z_phi_theta_psi
        xRot = Rotation.from_euler('x', [x_y_z_phi_theta_psi[5]]).as_matrix()
        yRot = Rotation.from_euler('y', [x_y_z_phi_theta_psi[4]]).as_matrix()
        zRot = Rotation.from_euler('z', [x_y_z_phi_theta_psi[3]]).as_matrix()
        R = np.squeeze(np.dot(np.dot(zRot, yRot), xRot))
        r11 = R[0, 0]
        r12 = R[0, 1]
        r13 = R[0, 2]

        r21 = R[1, 0]
        r22 = R[1, 1]
        r23 = R[1, 2]

        r31 = R[2, 0]
        r32 = R[2, 1]
        r33 = R[2, 2]

        xc = x_y_z_phi_theta_psi[0] - self.DH['d_6'] * R[0, 2]
        yc = x_y_z_phi_theta_psi[1] - self.DH['d_6'] * R[1, 2]
        zc = x_y_z_phi_theta_psi[2] - self.DH['d_6'] * R[2, 2]
        if self.logging:
            logger.debug(f'xc = {xc}, yc = {yc}, zc = {zc}')
        d1 = self.DH['d_1']
        a2 = self.DH['a_2']  # a2 и a3 по Спонгу - длины второго и третьего плеча
        a3 = self.DH['d_4']
        r = math.sqrt(xc ** 2 + yc ** 2) - self.DH['a_1']
        s = zc - d1
        D = (r ** 2 + s ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        try:
            if not theta3plus:
                theta_3 = atan2(sqrt(1 - D ** 2), D)
                theta_2 = atan2(s, r) + atan2(a3 * sin(theta_3), a2 + a3 * cos(theta_3))
                theta_3 = -theta_3  # инвертирование угла необходимо, так как в эту сторону мы вращаемся отрицательно
            else:
                theta_3 = atan2(sqrt(1 - D ** 2), D)
                theta_2 = atan2(s, r) - atan2(a3 * sin(theta_3), a2 + a3 * cos(theta_3))
        except:
            logger.error(ValueError)
            return [0, 0, 0, 0, 0, 0]
        theta_1 = atan2(yc, xc)
        # # Сферическое запястье
        cja = [theta_1, theta_2,
               theta_3]
        T = []
        for i in range(len(cja)):
            d = self.DH[f'displacement_theta_{i + 1}']
            T.append(np.array(
                [[cos(cja[i] + d), -sin(cja[i] + d) * cos(self.DH[f'alpha_{i + 1}']),
                  sin(cja[i] + d) * sin(self.DH[f'alpha_{i + 1}'])],
                 [sin(cja[i] + d), cos(cja[i] + d) * cos(self.DH[f'alpha_{i + 1}']),
                  -cos(cja[i] + d) * sin(self.DH[f'alpha_{i + 1}'])],
                 [0, sin(self.DH[f'alpha_{i + 1}']), cos(self.DH[f'alpha_{i + 1}'])]]))
        theta_5 = atan2(-sqrt(1 - (R[0, 2] * cos(theta_1) * cos(theta_2 + theta_3)
                                   + R[1, 2] * sin(theta_1) * cos(theta_2 + theta_3)
                                   + R[2, 2] * sin(theta_2 + theta_3)) ** 2),
                        R[0, 2] * cos(theta_1) * cos(theta_2 + theta_3) +
                        R[1, 2] * sin(theta_1) * cos(theta_2 + theta_3) + R[2, 2] * sin(theta_2 + theta_3))
        theta_4 = atan2(
            -(r13 * sin(theta_1) - r23 * cos(theta_1)) / sqrt(1 - (r13 * cos(theta_1) * cos(theta_2 + theta_3) +
                                                                   r23 * sin(theta_1) * cos(theta_2 + theta_3) +
                                                                   r33 * sin(theta_2 + theta_3)) ** 2),
            sqrt(1 - (r13 * sin(theta_1) - r23 * cos(theta_1)) ** 2 / (1 - (r13 * cos(theta_1) * cos(theta_2 + theta_3)
                                                                            + r23 * sin(theta_1) * cos(theta_2 +
                                                                                                       theta_3) +
                                                                            r33 * sin(theta_2 + theta_3)) ** 2)))
        theta_6 = atan2((r12 * cos(theta_1) * cos(theta_2 + theta_3) + r22 * sin(theta_1) * cos(theta_2 + theta_3) +
                         r32 * sin(theta_2 + theta_3)) / sqrt(1 - (r13 * cos(theta_1) * cos(theta_2 + theta_3) +
                                                                   r23 * sin(theta_1) * cos(
                    theta_2 + theta_3) + r33 * sin(theta_2 + theta_3)) ** 2),
                        sqrt(((r12 * cos(theta_1) * cos(theta_2 + theta_3) + r22 * sin(theta_1) * cos(
                            theta_2 + theta_3) +
                               r32 * sin(theta_2 + theta_3)) ** 2 + (r13 * cos(theta_1) * cos(theta_2 + theta_3) +
                                                                     r23 * sin(theta_1) * cos(
                                    theta_2 + theta_3) + r33 * sin(theta_2 +
                                                                   theta_3)) ** 2 - 1) / ((r13 * cos(theta_1) * cos(
                            theta_2 + theta_3) + r23 * sin(theta_1) * cos(theta_2 +
                                                                          theta_3) + r33 * sin(
                            theta_2 + theta_3)) ** 2 - 1)))
        self.last_inverse_pos = x_y_z_phi_theta_psi
        self.save_position()
        return [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

    def take_coordinate(self, array_matrix, number_of_matrix1, number_of_matrix2):
        matrix = self.matrix_dot(array_matrix, number_of_matrix1, number_of_matrix2)
        vector_xyz = matrix[0:3, 3]
        return vector_xyz

    def take_rotation_matrix(self, array_matrix, number_of_matrix1, number_of_matrix2):
        matrix = self.matrix_dot(array_matrix, number_of_matrix1, number_of_matrix2)
        rotation_matrix = matrix[0:3, 0:3]
        return rotation_matrix

    def anti_zero(self):
        for joint in self.joints:
            if joint.current_joint_angle == 0:
                joint.current_joint_angle = 0.00001

    def check_all(self):
        for i in self.joints:
            print(f"{i} -> {i.current_joint_angle}")
        print(f"x -> {self.position.x}")
        print(f"y -> {self.position.y}")
        print(f"z -> {self.position.z}")
        print(f"theta -> {self.position.theta}")
        print(f"phi -> {self.position.phi}")
        print(f"psi -> {self.position.psi}")

    def print(self):
        logger.debug(
            f"x = {self.position.x} y = {self.position.y} z = {self.position.z} \n theta "
            f"= {np.degrees(self.position.theta)} phi = {np.degrees(self.position.phi)}"
            f" psi = {np.degrees(self.position.psi)}")
        for i in range(6):
            logger.debug(f"joint number {i + 1} have angle = {self.joints[i].current_joint_angle}")

        for i in range(6):
            logger.debug(f"joint number {i + 1} have current step = {self.joints[i].current_joint_step}")

    def show(self):
        self.calculate_direct_kinematics_problem()
        self.robot.show()

    def move_xyz(self, pos, relative_angles=False):
        pos = np.array(pos)
        len = pos.shape[0]
        if len == 3:
            pos = np.hstack([pos, [0, pi, 0]])
        if len == 2:
            pos = np.hstack([pos, [self.position.z, 0, pi, 0]])
        if relative_angles == True:
            logger.debug("relative_angle is True")
        # pos = np.array([pos[0] / 1000, pos[1] / 1000, pos[2] / 1000,
        #                0, pi, 0])
        if (self.logging == True):
            logger.debug(pos)
        #logger.debug(pos)
        need_angles = self.calculate_inverse_kinematic_problem(pos)
        need_angles = np.degrees(need_angles)
        self.jog_joints(need_angles)
        self.calculate_direct_kinematics_problem()

    def move_all_xyz(self, xyz):
        lenth_x = xyz[0]
        lenth_y = xyz[1]
        lenth_z = xyz[2]
        # logger.debug(lenth_y)
        if len(self.last_inverse_pos) != 0:
            position = self.last_inverse_pos
        else:
            self.calculate_direct_kinematics_problem()
            position = [self.position.x, self.position.y, self.position.z,
                        self.position.theta, self.position.phi, self.position.psi]
            if self.logging:
                logger.debug(f"position in direct if = {position}")
        if self.logging:
            logger.debug(position)
        position[0] = position[0] + lenth_x
        position[1] = position[1] + lenth_y
        self.move_xyz(position)

    def move_x(self, lenth_x):  # принимаем мм
        lenth_x = lenth_x / 1000
        position = self.last_inverse_pos
        position[0] = position[0] + lenth_x
        if self.logging:
            logger.debug(position)
        self.move_xyz(position)

    def move_y(self, lenth_y):
        lenth_y = lenth_y / 1000
        position = self.last_inverse_pos
        position[1] = position[1] + lenth_y
        if self.logging:
            logger.debug(position)
        self.move_xyz(position)

    def move_z(self, lenth_z):
        lenth_z = lenth_z / 1000
        position = self.last_inverse_pos
        position[2] = position[2] + lenth_z
        if self.logging:
            logger.debug(position)
        self.move_xyz(position)

    def move_theta(self, lenth_theta):
        lenth_theta = np.radians(lenth_theta)
        position = self.last_inverse_pos
        position[3] = position[3] + lenth_theta
        if self.logging:
            logger.debug(position)
        self.move_xyz(position)

    def move_phi(self, lenth_phi):
        lenth_theta = np.radians(lenth_phi)
        position = self.last_inverse_pos
        position[4] = position[4] + lenth_phi
        if self.logging:
            logger.debug(position)
        self.move_xyz(position)

    def move_psi(self, lenth_psi):
        lenth_theta = np.radians(lenth_psi)
        position = self.last_inverse_pos
        position[5] = position[5] + lenth_psi
        if self.logging:
            logger.debug(position)
        self.move_xyz(position)

    def grab(self):
        command = f"SV{0}P{0}\n"
       # if (self.logging == True):
            #logger.debug(command)
        self.arduino_push(command)

    def absolve(self):
        command = f"SV{0}P{170}\n"
        #if (self.logging == True):
           # logger.debug(command)
        self.arduino_push(command)

    def rotate_3(self, vectors, axis, angle):  # [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
        k = []
        v_norm = axis / np.linalg.norm(axis)
        R = Rotation.from_rotvec(angle * v_norm).as_matrix()
        for i in range(3):
            v = vectors[i]
            k.append(np.dot(v, R))
        return k

    def ZYX_transform(self, angles):
        # angles = [pi/2, 0, 0]
        rotateX = np.array([[1, 0, 0],
                            [0, np.cos(angles[0]), -np.sin(angles[0])],
                            [0, np.sin(angles[0]), np.cos(angles[0])]])
        rotateY = np.array([[np.cos(angles[1]), 0, np.sin(angles[1])],
                            [0, 1, 0],
                            [-np.sin(angles[1]), 0, np.cos(angles[1])]])
        rotateZ = np.array([[np.cos(angles[2]), -np.sin(angles[2]), 0],
                            [np.sin(angles[2]), np.cos(angles[2]), 0],
                            [0, 0, 1]])
        T = np.dot(rotateZ, rotateY).dot(rotateX)
        return T

    def RZYZ_transform(self, angles):
        rotateZ = np.array([[np.cos(angles[0]), -np.sin(angles[0]), 0],
                            [np.sin(angles[0]), np.cos(angles[0]), 0],
                            [0, 0, 1]])
        rotateY = np.array([[np.cos(angles[1]), 0, np.sin(angles[1])],
                            [0, 1, 0],
                            [-np.sin(angles[1]), 0, np.cos(angles[1])]])
        rotateZ2 = np.array([[np.cos(angles[2]), -np.sin(angles[2]), 0],
                             [np.sin(angles[2]), np.cos(angles[2]), 0],
                             [0, 0, 1]])
        T = np.dot(rotateZ, rotateY).dot(rotateZ2)
        return T

    def matrixRZYZ(self, angles_xyz):
        angles_xyz = np.array(angles_xyz)
        rotateZ = np.array([[np.cos(angles_xyz[0]), -np.sin(angles_xyz[0]), 0],
                            [np.sin(angles_xyz[0]), np.cos(angles_xyz[0]), 0],
                            [0, 0, 1]])
        rotateY = np.array([[np.cos(angles_xyz[1]), 0, np.sin(angles_xyz[1])],
                            [0, 1, 0],
                            [-np.sin(angles_xyz[1]), 0, np.cos(angles_xyz[1])]])
        rotateZ2 = np.array([[np.cos(angles_xyz[2]), -np.sin(angles_xyz[2]), 0],
                             [np.sin(angles_xyz[2]), np.cos(angles_xyz[2]), 0],
                             [0, 0, 1]])
        T = np.dot(rotateZ, rotateY).dot(rotateZ2)
        angles = np.array([angles_xyz[3:6]])
        angles = angles.transpose()
        matrix = np.hstack([T, angles])
        T1 = np.vstack([matrix, [0, 0, 0, 1]])
        return T1

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return [roll_x, pitch_y, yaw_z]

    def rotate_from_angle(self, angle, axis, l4x4=False):
        if (l4x4 == False):
            if (axis == 'x'):
                rotate = np.array([[1, 0, 0],
                                   [0, np.cos(angle), -np.sin(angle)],
                                   [0, np.sin(angle), np.cos(angle)]])

            if (axis == 'y'):
                rotate = np.array([[np.cos(angle), 0, np.sin(angle)],
                                   [0, 1, 0],
                                   [-np.sin(angle), 0, np.cos(angle)]])
            if (axis == 'z'):
                rotate = np.array([[np.cos(angle), -np.sin(angle), 0],
                                   [np.sin(angle), np.cos(angle), 0],
                                   [0, 0, 1]])
        if (l4x4 == True):
            if (axis == 'x'):
                rotate = np.array([[1, 0, 0, 0],
                                   [0, np.cos(angle), -np.sin(angle), 0],
                                   [0, np.sin(angle), np.cos(angle), 0],
                                   [0, 0, 0, 1]])

            if (axis == 'y'):
                rotate = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                                   [0, 1, 0, 0],
                                   [-np.sin(angle), 0, np.cos(angle), 0],
                                   [0, 0, 0, 1]])
            if (axis == 'z'):
                rotate = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                                   [np.sin(angle), np.cos(angle), 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
        return rotate

    def trans(self, xyzabc):

        angle_z = pi / 2
        angle_x = pi
        # смещение относительно системы координат камеры
        xc = 0  # -0.03
        yc = 0  # -0.03 + 0.1
        zc = 0.
        # logger.debug(f'xyzabc[1] = {xyzabc[1]}')
        xyzabc[0] = xyzabc[0] + xc
        xyzabc[1] = xyzabc[1] + yc
        # logger.debug(f'xyzabc[1] = {xyzabc[1]}')
        xyzabc[2] = xyzabc[2] + zc
        # смещение относительно системы координат схвата
        x = 0
        y = 0
        z = 0
        Td = np.array([[1, 0, 0, x],
                       [0, 1, 0, y],
                       [0, 0, 1, z],
                       [0, 0, 0, 1]])
        Tz = self.rotate_from_angle(angle_z, 'z', True)
        Tx = self.rotate_from_angle(angle_x, 'x', True)
        # logger.debug(Tz)
        # logger.debug(Tx)
        T = np.dot(Td, Tz)
        # logger.debug(f"Td*Tz {T}")
        T = np.dot(T, Tx)
        # logger.debug(f"Td*Tz*Tx {T[0:3, 0:3]}")
        vector_xyz = np.array([xyzabc[0],
                               xyzabc[1],
                               xyzabc[2]])
        # logger.debug(vector_xyz)
        # Координаты арукомаркера относительно системы камеры
        coordinates = np.dot(vector_xyz, T[0:3, 0:3])
        # logger.debug(coordinates)
        coordinates[2] = - coordinates[2]  # инвертируем ось z, потому что другая тройка векторов
        return coordinates

        # T6_7_z = np.array([[cos(theta), -sin(theta), 0, x * cos(theta)],
        #                  [sin(theta), cos(theta), 0, y * sin(theta)],
        #                  [0, 0, 1, z],
        #                  [0, 0, 0, 1]])
        # T6_7_x = np.array([[1, 0, 0, 0],
        #                  [0, cos(phi), -sin(phi), 0],
        #                  [0, sin(phi), cos(phi), 0],
        #                  [0, 0, 0, 1]])
        # T6_7 = np.dot(T6_7_z, T6_7_x)
        # T0_6 = self.matrix_dot(self.calculate_direct2(), 0, 6)
        # angles = self.angular_Euler_calculation(T0_6[0:3, 0:3])
        # # logger.debug(np.degrees(angles))
        # T0_7 = np.dot(T0_6, T6_7)
        # # logger.debug(T0_7)
        # angles = self.angular_Euler_calculation(T0_7[0:3, 0:3])
        # # logger.debug(np.degrees(angles))
        # return [T6_7[0, 3], T6_7[1, 3], T6_7[2, 3]]

    def camera_init(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        return cap

    def openCV(self, id_camera, id_marker=0, max_wait=50):
        # self.move_xyz([0.28683, 0.1, 0.05, 0, pi, 0])
        aruco_marker_side_length = 0.0344
        aruco_dictionary_name = "DICT_4X4_50"
        camera_calibration_parameters_filename = 'calibration_chessboardDEXP1080.yaml'
        cap = self.cap
        odom = arucoOdometry.arucoOdometry()
        odom.setCameraParams(camera_calibration_parameters_filename)
        odom.setArucoLength(aruco_marker_side_length)
        odom.setArucoDict(aruco_dictionary_name)
        markers = [
            {"id": id_marker, "size": aruco_marker_side_length}]  # , {"id": id2, "size": aruco_marker_side_length}]
        odom.setMarkers(markers)

        startTime = time.time() * 1000
        cycle = 5
        array = np.array([0, 0, 0, 0, 0, 0])
        i = 0
        logger.debug("begin cycle")
        waitnumber = 0
        while (True):
            ret, frame = cap.read()
            frame, x, y, z, a_x, a_y, a_z = odom.updateCameraPoses(frame, time.time() * 1000 - startTime, id_marker)
            cv2.imshow("im", frame)
            cv2.waitKey(1)
            waitnumber += 1
            if self.logging:
                logger.debug(f"waitkey = {waitnumber}")
            if waitnumber > max_wait:
                logger.debug("Маркер не обнаружен")
                break
            # y -= 0.1
            # logger.debug(x)
            if not x == 0 or not y == 0 or not z == 0:
                i += 1
                xyz = np.array([x, y, z, a_x, a_y, a_z])
                array = np.vstack([array, xyz])

                # logger.debug(xyz)
                # logger.debug(f'------array = {array}')
            if i > cycle:
                break

            #     logger.debug(f'1 x = {x} y = {y} z = {z}')
            #     xyz = self.trans([x, y, z, a_x, a_y, a_z])
            #     logger.debug(f'2 ---- x = {xyz[0]} y = {xyz[1]} z = {xyz[2]}')
            # xyz[0] = -xyz[0]
            # xyz[1] = -xyz[1]

            # np.vstack([array, xyz])
        if (waitnumber > 100):
            return None

        array = np.delete(array, 0, 0)
        logger.debug(array)
        mean_array = np.mean(array, axis=0)
        logger.debug(mean_array)
        # смещение по оси y камеры
        # mean_array[1] -= 0.02
        coord = self.trans(mean_array)
        logger.debug(coord)
        coord = np.hstack([coord, [a_x, a_y, a_z]])
        logger.debug(coord)
        # self.move_all_xyz([coord[0], coord[1], 0])
        # coord[0] += 0.05
        # coord[1] -= 0.013
        return coord

    def openCV2(self, id_camera, id_marker0=11, id_marker1=12):
        # self.move_xyz([0.28683, 0.1, 0.05, 0, pi, 0])
        aruco_marker_side_length = 0.0344
        aruco_dictionary_name = "DICT_4X4_50"
        camera_calibration_parameters_filename = 'calibration_chessboardDEXP1080.yaml'
        cap = self.cap
        odom = arucoOdometry.arucoOdometry()
        odom.setCameraParams(camera_calibration_parameters_filename)
        odom.setArucoLength(aruco_marker_side_length)
        odom.setArucoDict(aruco_dictionary_name)
        markers = [
            {"id": id_marker0, "size": aruco_marker_side_length}, {"id": id_marker1, "size": aruco_marker_side_length}]
        odom.setMarkers(markers)

        startTime = time.time() * 1000
        cycle = 5
        array0 = np.array([0, 0, 0, 0, 0, 0])
        array1 = np.array([0, 0, 0, 0, 0, 0])
        i = 0
        #logger.debug("begin cycle")
        while (True):
            ret, frame = cap.read()
            massive = odom.updateCameraPoses2(frame,
                                              time.time() * 1000 - startTime,
                                              [id_marker0, id_marker1])
            frame = massive[0]
            # logger.debug(massive[1])
            xyzabc0 = np.array(massive[1][0])
            # logger.debug(xyzabc0)
            xyzabc1 = np.array(massive[1][1])
            cv2.imshow("im", frame)
            cv2.waitKey(1)

            if not xyzabc0[0] == 0 or not xyzabc0[1] == 0 or not xyzabc0[2] == 0 or not xyzabc1[0] == 0 \
                    or not xyzabc1[1] == 0 or not xyzabc1[2] == 0:
                i += 1
                array0 = np.vstack([array0, xyzabc0])
                array1 = np.vstack([array1, xyzabc1])
            if i > cycle:
                break
        array0 = np.delete(array0, 0, 0)
        array1 = np.delete(array1, 0, 0)
        mean_array0 = np.mean(array0, axis=0)
        mean_array1 = np.mean(array1, axis=0)
        # смещение по оси y камеры
        #logger.debug(f'until = {mean_array0[1]}')
        # mean_array0[1] += 0.04
        # mean_array1[1] += 0.04
        #logger.debug(f'after = {mean_array0[1]}')
        coord0 = self.trans(mean_array0)
        coord1 = self.trans(mean_array1)

        coord0 = np.hstack([coord0, [mean_array0[3], mean_array0[4], mean_array0[5]]])
        coord1 = np.hstack([coord1, [mean_array1[3], mean_array1[4], mean_array1[5]]])
        # logger.debug(f'coord0 = {coord0}')
        # logger.debug(f'coord1 = {coord1}')

        coord0[0] += 0.05
        coord1[0] += 0.05

        coord0[1] -= 0.013
        coord1[1] -= 0.013

        return coord0, coord1

    def camera_calibrate(self, number):
        d = 0.03
        xyz_0 = self.openCV(0, number)
        if xyz_0.all() == None:
            return None
        xyz_0[0] += 0.05
        xyz_0[1] -= 0.013
        current_coord = self.calculate_direct_kinematics_problem()
        # координаты предварительно вычесленного центра
        x0 = current_coord[0] + xyz_0[0]
        y0 = current_coord[1] + xyz_0[1]
        z0 = 0.25
        #logger.debug(f'x0 = {x0}, y0 = {y0}')
        D1 = [x0, y0, z0]
        time.sleep(1)
        self.move_xyz(D1)
        time.sleep(2)
        #######################

        xyz_0 = self.openCV(0, number)
        xyz_0[0] += 0.05
        xyz_0[1] -= 0.013
        current_coord = self.calculate_direct_kinematics_problem()
        # координаты предварительно вычесленного центра
        x0 = current_coord[0] + xyz_0[0]
        y0 = current_coord[1] + xyz_0[1]
        z0 = 0.15
        #logger.debug(f'x0 = {x0}, y0 = {y0}')
        D1 = [x0, y0, z0]
        time.sleep(1)
        self.move_xyz(D1)
        time.sleep(2)
        #######################

        xyz_0 = self.openCV(0, number)
        xyz_0[0] += 0.05
        xyz_0[1] -= 0.013
        current_coord = self.calculate_direct_kinematics_problem()
        # координаты предварительно вычесленного центра
        x0 = current_coord[0] + xyz_0[0]
        y0 = current_coord[1] + xyz_0[1]
        z0 = 0.07
        #logger.debug(f'x0 = {x0}, y0 = {y0}')
        D1 = [x0, y0, z0]
        time.sleep(1)
        self.move_xyz(D1)
        time.sleep(2)

        for i in range(4):
            xyz_0 = self.openCV(0, number)
            xyz_0[0] += 0.05
            xyz_0[1] -= 0.013
            current_coord = self.calculate_direct_kinematics_problem()
            # координаты предварительно вычесленного центра
            x0 = current_coord[0] + xyz_0[0]
            y0 = current_coord[1] + xyz_0[1]
            z0 = 0.04
            #logger.debug(f'x0 = {x0}, y0 = {y0}')
            D1 = [x0, y0, z0]
            time.sleep(1)
            self.move_xyz(D1)
            time.sleep(0.1)

    def camera_calibrate_s(self, number, hight, count=1, state=0, delta=None):  # state = 0 or 1
        if delta is None:
            delta = [0, 0, 0]
        for i in range(count):
            xyz_0 = self.openCV(0, number)
            current_coord = self.calculate_direct_kinematics_problem()
            # Смещение для выравнивания по центру
            delta_x = float(delta[0])
            delta_y = float(delta[1])
            delta_z = float(delta[2])
            # координаты предварительно вычесленного центра

            if state == 0:
                a = pi
                b = pi
                c = 0
                x0 = current_coord[0] + xyz_0[0] + delta_x
                y0 = current_coord[1] + xyz_0[1] + delta_y
                z0 = hight / 1000
            elif state == 1:
                a = 0
                b = pi
                c = 0
                x0 = current_coord[0] - xyz_0[0] - delta_x
                y0 = current_coord[1] - xyz_0[1] - delta_y
                z0 = hight / 1000
            else:
                a = pi
                b = pi
                c = 0
                x0 = current_coord[0] + xyz_0[0] + delta_x
                y0 = current_coord[1] + xyz_0[1] + delta_y
                z0 = hight / 1000
            #logger.debug(f'x0 = {x0}, y0 = {y0}')
            D1 = [x0, y0, z0, a, b, c]
            self.move_xyz(D1)
            time.sleep(1)

    def camera_calibrate2(self):
        d = 0.003
        # Val = True
        for i in range(5):
            xyz_0, xyz_1 = self.openCV2(0, 11, 12)
            #logger.debug(xyz_0)
            # print(1)
            current_coord = self.calculate_direct_kinematics_problem()
            # # координаты предварительно вычесленного центра для двух арукомаркеров
            x0 = current_coord[0] + xyz_0[0]
            x1 = current_coord[0] + xyz_1[0]

            y0 = current_coord[1] + xyz_0[1]
            y1 = current_coord[1] + xyz_1[1]

            # вычислим центр между двумя арукомаркерами C (x_c, y_c)

            x_c = (x0 + x1) / 2
            y_c = (y0 + y1) / 2
            z_0 = 0.15  # current_coord[2] - xyz_0[2] + 0.25
            #
            point = [x_c, y_c, z_0]
            self.move_xyz(point)
            time.sleep(0.1)
            # if abs(x_c - current_coord[0]) < d and abs(y_c - current_coord[1]):
            #     break
        # xy0 = np.array([x0, y0])
        # logger.debug(f'x0 = {x0}, y0 = {y0}')
        # D1 = [x0, y0 - d, z0]
        # D3 = [x0, y0 + d]
        # D2 = [x0 - d, y0]
        # # D3 = [x0, y0 + d]
        # D4 = [x0 + d, y0]
        # xy_massive = []
        # # Калибровка оси x
        # self.move_xyz(D1)
        # time.sleep(5)
        # d1 = self.openCV(0, 11)
        # xy_massive.append([self.position.x + d1[0], self.position.y + d1[1]])
        #
        # self.move_xyz(D3)
        # time.sleep(2)
        # d3 = self.openCV(0, 11)
        # xy_massive.append([self.position.x + d3[0], self.position.y + d3[1]])
        #
        # # Калибровка оси y
        # self.move_xyz(D2)
        # time.sleep(2)
        # d2 = self.openCV(0, 11)
        # xy_massive.append([self.position.x + d2[0], self.position.y + d2[1]])
        #
        # self.move_xyz(D4)
        # time.sleep(2)
        # d4 = self.openCV(0, 11)
        # xy_massive.append([self.position.x + d4[0], self.position.y + d4[1]])
        #
        # xy_mean = np.mean(xy_massive, axis=0)
        # logger.debug(xy_mean)
        # xy_mean[0] -= 0.1
        # self.move_xyz(xy_mean)

    def take_object(self):
        self.move_xyz([self.position.x - 0.04895, self.position.y - 0.001, self.position.z - 0.008])
        # self.move_xyz([self.position.x, self.position.y, self.position.z - 0.12])
        # self.move_xyz([0.24134, -0.12835, 0.03, 0, pi, 0])

    # x = 0.24134393860523998 y = -0.12435046709099302 z = 0.030000000000000082
    def camera_calibrate_rot(self, number):
        for i in range(7):
            xyz_0 = self.openCV(0, number)
            xyz_0[0] -= 0.007
            xyz_0[1] -= 0.018
            current_coord = self.calculate_direct_kinematics_problem()
            # координаты предварительно вычесленного центра
            x0 = current_coord[0]
            y0 = current_coord[1]
            z0 = 0.25
            a_z = self.last_inverse_pos[3]

            vect = np.array([[xyz_0[0]],
                             [xyz_0[1]],
                             [z0]])
            rot_m = self.rotate_from_angle(a_z, 'z')

            new_coord = np.dot(rot_m, vect)
            #logger.debug(f"xyz_0 = {xyz_0}")
            #logger.debug(f"new_coord = {new_coord}")
            x0 = current_coord[0] + new_coord[0][0]
            y0 = current_coord[1] + new_coord[1][0]

            D1 = [x0, y0, self.position.z, pi / 2, pi, 0]
            self.move_xyz(D1, True)
            time.sleep(1)

    # def getRobotPosition():
    #     commandCalc = "GP" + "U" + str(J1StepCur) + "V" + str(J2StepCur) + "W" + str(J3StepCur) + "X" + str(
    #         J4StepCur) + "Y" + str(J5StepCur) + "Z" + str(J6StepCur) + "\n"
    #     ser.write(commandCalc.encode())
    #     RobotCode = str(ser.readline())
    #     Pcode = RobotCode[2:4]
    #     if (Pcode == "01"):
    #         applyRobotCal(RobotCode)
    def enc(self):
        command = "TT"
        self.serial_teensy.flushInput()
        self.teensy_push(command)
        # time.sleep(2)
        # logger.debug(self.serial_teensy.in_waiting)
        #
        myBytes = self.serial_teensy.read_all()

        # bufferBytes = self.serial_teensy.inWaiting()
        # #
        # # # If exists, it is added to the myBytes variable with previously read information
        # if bufferBytes:
        #      myBytes += self.serial_teensy.read(bufferBytes)
        logger.debug(f"input = {myBytes}")
