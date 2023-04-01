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
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import plotly.graph_objects as go
from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *


# from parse import parse


@dataclass(init=True)
class Position:
    """
    Class for keeping global position and orientation of manipulitor's wrist
    """
    x: float = 0  # mm
    x_m = x / 1000  # m
    y: float = 0  # mm
    y_m = y / 1000  # m
    z: float = 0  # mm
    z_m = z / 1000  # m
    theta: float = 0  # grad
    theta_rad = theta * pi / 180  # rad
    phi: float = 0  # grad
    phi_rad = phi * pi / 180  # rad
    psi: float = 0  # grad
    psi_rad = psi * pi / 180  # rad
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
        'd_6': 0.125,  # 0.03625,
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

    def __init__(self, teensy_port, arduino_port, baud):
        self.showMode = False
        self.robot = RobotSerial(self.dh_params)
        self.last_matrix = []
        self.ijk = np.array([])
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
        self.limits = np.radians(
            np.array([[self.joints[0].negative_angle_limit, self.joints[0].positive_angle_limit],
                      [self.joints[1].negative_angle_limit, self.joints[1].positive_angle_limit],
                      [self.joints[2].positive_angle_limit, self.joints[2].negative_angle_limit],
                      [self.joints[3].negative_angle_limit, self.joints[3].positive_angle_limit],
                      [self.joints[4].negative_angle_limit, self.joints[4].positive_angle_limit],
                      [self.joints[5].positive_angle_limit, self.joints[5].negative_angle_limit],
                      ]))
        self.robot.ws_lim = self.limits
        self.calculate_direct_kinematics_problem()
        try:
            self.serial_teensy: serial.Serial = serial.Serial(teensy_port, baud)
            self.serial_arduino: serial.Serial = serial.Serial(arduino_port, baud)
            self.is_connected = True
        except serial.SerialException:
            logger.error("Serial port not defined")

    def show_workspace(self):
        self.robot.ws_division = 6
        self.robot.show(ws=True)

    def save_position(self):
        logger.debug("Запись в файл")
        file = open("lastPos", "w")
        file.truncate()
        file.write(
            f"{self.joints[0].current_joint_angle}, {self.joints[1].current_joint_angle}, "
            f"{self.joints[2].current_joint_angle}, {self.joints[3].current_joint_angle}, "
            f"{self.joints[4].current_joint_angle}, {self.joints[5].current_joint_angle}")
        file.close()

    def restore_position(self):
        file = open("lastPos", "r")
        text = file.read()
        angles = text.split(",")
        for i, angle in enumerate(angles):
            angle = float(angle)
            self.joints[i].current_joint_angle = angle

    def finish(self):
        self.serial_teensy.close()
        self.serial_arduino.close()

    def write_point(self, string):
        file = open("points.txt", "w")
        file.write(string)
        file.close()

    def read_points(self):
        file = open("points.txt", "r")
        commands = file.read()
        command = commands.split("\n")
        for i in range(len(command)):
            # if (command[i] == "grab"):
            angles = command[i].split(",")
            mas = []
            for g in range(len(angles)):
                if (angles[g] == "grab"):
                    logger.debug("grab------------------------------------->")
                    self.grab()
                elif (angles[g] == "abs"):
                    logger.debug("absolve------------------------------------->")
                    self.absolve()
                elif (angles[g] == "sleep"):
                    logger.debug("sleep------------------------------------->")
                    time.sleep(float(angles[g + 1]))
                    self.absolve()
                else:
                    mas.append(float(angles[g]))
            if (len(mas) > 2):
                self.jog_joints(mas)

    def get_joints_angles(self) -> list:
        return [joint.current_joint_angle for joint in self.joints]

    def move_to(self, command):
        self.serial_teensy.write(command.encode())

    def info(self):
        for i in range(6):
            print(f"Отрицательный лимит {i + 1}го звена: {DEFAULT_SETTINGS[f'J{i + 1}_negative_angle_limit']} \n"
                  f"Положительный лимит {i + 1}го звена: {DEFAULT_SETTINGS[f'J{i + 1}_positive_angle_limit']} \n"
                  f"\n")

    def calc_angle(self, angle, joint: Joint):
        angle = float(angle)
        if (joint.positive_angle_limit > 0):
            if (angle > joint.positive_angle_limit or angle < joint.negative_angle_limit):
                logger.error(f"Угол звена {joint.get_name_joint()} превышает лимит")
                return [0, 0, True]
        if (joint.positive_angle_limit < 0):
            if (angle < joint.positive_angle_limit or angle > joint.negative_angle_limit):
                logger.error(f"Угол звена {joint.get_name_joint()} превышает лимит")
                return [0, 0, True]
        # Расчет направления двигателей
        x = joint.current_joint_angle
        arc = abs(angle - x)
        if (joint.motor_dir == 1):
            if (angle > x):
                drive_direction = 0
            if (angle < x):
                drive_direction = 1
            if (angle == x):
                logger.error(f"Звено {joint.get_name_joint()} уже в этом положении")
                drive_direction = 1
                arc = 0
        if (joint.motor_dir == -1):
            if (angle > x):
                drive_direction = 1
            if (angle < x):
                drive_direction = 0
            if (angle == x):
                logger.error(f"Звено {joint.get_name_joint()} уже в этом положении")
                drive_direction = 1
                arc = 0
        error = False
        return [arc, drive_direction, error]

    def jog_joint_c(self, joint: Joint, degrees):
        d = self.calc_angle(degrees, joint)
        logger.debug(f"angle, d = {d}")
        if (d[1] == 1):
            self.jog_joint(joint, self.position.speed, d[0])
        if (d[1] == 0):
            self.jog_joint(joint, self.position.speed, -d[0])

    def jog_joints(self, degrees):
        degrees = [float(x) for x in degrees]
        joint_commands = []
        errors = []
        angles = []
        for i in range(6):
            d = self.calc_angle(degrees[i], self.joints[i])
            arc = d[0]
            logger.debug(self.joints[i].motor_dir)
            direction = d[1]
            j_jog_steps = abs(int(arc / self.joints[i].degrees_per_step))
            joint_commands.append(f"{self.joints[i].get_name_joint()}{direction}{j_jog_steps}")
            errors.append(d[2])
            # if (d[2] != True):
            #     logger.debug(f"Запись в джойнт {i+1}")
            angles.append(degrees[i])
            # self.joints[i].current_joint_angle = degrees[i]
        if (not errors[0] and not errors[1] and not errors[2] and not errors[3] and not errors[4] and not errors[5]):
            for i in range(6):
                self.joints[i].current_joint_angle = angles[i]
            command = f"MJ{''.join(joint_commands)}S{self.position.speed}G{15}H{10}I{20}K{5}\n"
            self.teensy_push(command)
            self.save_position()
            self.calculate_direct_kinematics_problem()
            logger.debug(f"Запись углов в джойнты: {angles}")
        else:
            logger.debug("Команда не отправилась, превышен лимит одного из джойнтов")

    def jog_joint(self, joint: Joint, speed, degrees):  # degrees - то, на сколько градусов мы двигаем Джойнт
        # Задача направления движения джойнта и отлов ошибок
        logger.debug("jog_joint")
        degrees = int(degrees)
        if not type(degrees) is int:
            raise TypeError("Only integer are allowed")

        if degrees < 0:
            drive_direction = 0
            degrees = abs(degrees)
        elif degrees > 0:
            drive_direction = 1
        else:
            raise NameError('Нет смысла двигаться на 0 градусов')

        if not self.JogStepsStat:  # JogStepsStat показывает, в каких единицах мы будем передвигать джойнт, либо в шагах, либо в градусах
            j_jog_steps = int(
                degrees / joint.degrees_per_step)  # высчитываем количенство шагов, joint.degrees_per_step -
        else:
            # switch from degs to steps
            j_jog_steps = degrees  # [град]
            degrees *= joint.degrees_per_step  # [град] * [град]/[шаг]

        axis_limit = False
        if drive_direction == 1:
            if degrees <= (joint.positive_angle_limit - joint.current_joint_angle):
                joint.current_joint_step += int(j_jog_steps)
            else:
                logger.warning(f"Joint {joint.number_joint} AXIS LIMIT")
                axis_limit = True
        if drive_direction == 0:
            if degrees <= -(joint.negative_angle_limit - joint.current_joint_angle):
                joint.current_joint_step -= int(j_jog_steps)
            else:
                logger.warning(f"Joint {joint.number_joint} AXIS LIMIT")
                axis_limit = True

        if not axis_limit:
            joint.current_joint_angle = round(
                joint.negative_angle_limit + (
                        joint.current_joint_step * joint.degrees_per_step))  # Jjogneg J1AngCur = round(J1NegAngLim + (J1StepCur * J1DegPerStep), 2)
            # J1AngCur = round(J1NegAngLim + (J1StepCur * J1DegPerStep), 2)
            self.save_data()
            # print(f"Новые координаты: {np.round(np.dot(self.calculate_direct_kinematics_problem(), 180 / pi), 3)}")
            j_jog_steps = int(round(j_jog_steps))
            command = f"MJ{joint.get_name_joint()}{drive_direction}{j_jog_steps}S{speed}G{self.ACC_dur}H{self.ACC_spd}" \
                      f"I{self.DEC_dur}K{self.DEC_spd}U{self.joints[0].current_joint_step}" \
                      f"V{self.joints[1].current_joint_step}W{self.joints[2].current_joint_step}" \
                      f"X{self.joints[3].current_joint_step}Y{self.joints[4].current_joint_step}" \
                      f"Z{self.joints[5].current_joint_step}\n"
            self.teensy_push(command)
            # logger.debug(f"Write to teensy: {command.strip()}")
            self.serial_teensy.flushInput()
            time.sleep(.2)
            self.calculate_direct_kinematics_problem2()
            robot_code = str(self.serial_teensy.readline())
            # if robot_code[2:4] == "01":
            # self.apply_robot_calibration(robot_code)

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
                    f'{joint.get_name_joint()} COLLISION OR OUT OF CALIBRATION')  # TODO: Ошибка выскакиевает при калибровке четвертого звена

                if i < 5:
                    joint.current_joint_step = int(robot_code[joint_name_indexes[i] + 1: joint_name_indexes[i + 1]])
                else:
                    joint.current_joint_step = int(robot_code[joint_name_indexes[i] + 1:])

                joint.current_joint_angle = round(joint.negative_angle_limit + (joint.current_joint_step *
                                                                                joint.degrees_per_step), 2)
                # self.stop_program()
        self.calculate_direct_kinematics_problem2()
        self.save_data()

    def teensy_push(self, command):
        logger.debug(f'Teensy push {command}')
        self.serial_teensy.write(command.encode())

    def arduino_push(self, command):
        logger.debug("")
        self.serial_arduino.write(command.encode())

    def save_data(self):
        for index, joint in enumerate(self.joints):
            DEFAULT_SETTINGS[f'J{index + 1}_current_step'] = joint.current_joint_step
            DEFAULT_SETTINGS[f'J{index + 1}_current_angle'] = joint.current_joint_angle
            DEFAULT_SETTINGS[f'J{index + 1}_negative_angle_limit'] = joint.negative_angle_limit
            DEFAULT_SETTINGS[f'J{index + 1}_positive_angle_limit'] = joint.positive_angle_limit
            DEFAULT_SETTINGS[f'J{index + 1}_step_limit'] = joint.step_limit
            DEFAULT_SETTINGS[f'J{index + 1}_open_loop_val'] = joint.open_loop_stat

        DEFAULT_SETTINGS['servo_0_on'] = None
        DEFAULT_SETTINGS['servo_0_off'] = None
        DEFAULT_SETTINGS['servo_1_on'] = None
        DEFAULT_SETTINGS['servo_1_off'] = None

        DEFAULT_SETTINGS['program_name'] = None

        DEFAULT_SETTINGS['DO_1_on'] = None
        DEFAULT_SETTINGS['DO_1_off'] = None
        DEFAULT_SETTINGS['DO_2_on'] = None
        DEFAULT_SETTINGS['DO_2_off'] = None

        DEFAULT_SETTINGS['UF_x'] = None
        DEFAULT_SETTINGS['UF_y'] = None
        DEFAULT_SETTINGS['UF_z'] = None
        DEFAULT_SETTINGS['UF_rx'] = None
        DEFAULT_SETTINGS['UF_ry'] = None
        DEFAULT_SETTINGS['UF_rz'] = None

        DEFAULT_SETTINGS['TF_x'] = None
        DEFAULT_SETTINGS['TF_y'] = None
        DEFAULT_SETTINGS['TF_z'] = None
        DEFAULT_SETTINGS['TF_rx'] = None
        DEFAULT_SETTINGS['TF_ry'] = None
        DEFAULT_SETTINGS['TF_rz'] = None

        DEFAULT_SETTINGS['fine_cal_position'] = None

        for i in range(6):
            DEFAULT_SETTINGS[f'DH_r_{i + 1}'] = None
        for i in range(6):
            DEFAULT_SETTINGS[f'DH_a_{i + 1}'] = None
        for i in range(6):
            DEFAULT_SETTINGS[f'DH_d_{i + 1}'] = None
        for i in range(6):
            DEFAULT_SETTINGS[f'DH_t_{i + 1}'] = None

        DEFAULT_SETTINGS['calibration_direction'] = None
        DEFAULT_SETTINGS['Mot_Dir'] = None

        DEFAULT_SETTINGS['Track_current'] = None
        DEFAULT_SETTINGS['Track_length'] = None
        DEFAULT_SETTINGS['Track_step_limit'] = None

        DEFAULT_SETTINGS['Vis_file_location'] = None
        # calibration.insert(tk.END, visoptions.get()) нужно узнать, что это такое
        DEFAULT_SETTINGS['Vis_Pic_OxP'] = None
        DEFAULT_SETTINGS['Vis_Pic_OxM'] = None
        DEFAULT_SETTINGS['Vis_Pic_OyP'] = None
        DEFAULT_SETTINGS['Vis_Pic_OyM'] = None

        DEFAULT_SETTINGS['Vis_Pic_XPE'] = None
        DEFAULT_SETTINGS['Vis_Pic_XME'] = None
        DEFAULT_SETTINGS['Vis_Pic_YPE'] = None
        DEFAULT_SETTINGS['Vis_Pic_YME'] = None

    @staticmethod
    def create_joints():
        joints_name = ['A', 'B', 'C', 'D', 'E', 'F']
        joints = [Joint(i + 1,
                        DEFAULT_SETTINGS[f'J{i + 1}_positive_angle_limit'],
                        DEFAULT_SETTINGS[f'J{i + 1}_negative_angle_limit'],
                        DEFAULT_SETTINGS[f'J{i + 1}_step_limit'])
                  for i in range(6)]

        for joint, joint_name in zip(joints, joints_name):
            joint.set_name_joint(joint_name)

        for i, joint in enumerate(joints):
            joint.current_joint_step = DEFAULT_SETTINGS[f'J{i + 1}_current_step']
            joint.current_joint_angle = DEFAULT_SETTINGS[f'J{i + 1}_current_angle']
            joint.motor_dir = DEFAULT_SETTINGS[f'J{i + 1}_dir']

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
        axes = [axis for axis in calibration_axes]

        steps = []
        for i, axis in enumerate(axes):
            if axis == '1':
                steps.append(self.joints[i].step_limit)
            else:
                steps.append(0)

        calibration_drive = self.get_calibration_drive()

        joint_calibration_drive_and_step = [f"{joint.get_name_joint()}{cd}{step}"
                                            for joint, cd, step in zip(self.joints, calibration_drive, steps)]
        command = f"LL{''.join(joint_calibration_drive_and_step)}S{speed}\n"
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()
        calibration_value = self.serial_teensy.read()
        if calibration_value == b'P':
            # calibration_status = 1
            for joint, cd, axis in zip(self.joints, self.calibration_direction, axes):
                if axis == '1':
                    if cd == '0':
                        if (joint.motor_dir == 1 or joint.motor_dir == 2):
                            joint.current_joint_step = 0
                            joint.current_joint_angle = joint.positive_angle_limit
                        else:
                            joint.current_joint_step = joint.step_limit
                            joint.current_joint_angle = joint.negative_angle_limit
                    else:
                        if (joint.motor_dir == -1):
                            joint.current_joint_step = joint.step_limit
                            joint.current_joint_angle = joint.negative_angle_limit
                        else:
                            joint.current_joint_step = 0
                            joint.current_joint_angle = joint.positive_angle_limit

            logger.success('CALIBRATION SUCCESSFUL')
        elif calibration_value == b'F':
            # calibration_status = 0
            logger.error('CALIBRATION FAILED')
        else:
            logger.warning('NO CAL FEEDBACK FROM ARDUINO')  # может быть ошибка

        self.calculate_direct_kinematics_problem()
        self.save_data()
        joints_current_steps = [f"{joint.get_name_joint()}{joint.current_joint_step}" for joint in self.joints]
        command = f'LM{"".join(joints_current_steps)}\n'
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()

    def auto_calibrate(self):
        self.calibrate('111111', '40')
        cd = self.get_calibration_drive_auto()  # направление калибровки
        command = f"MJA{cd[0]}500B{cd[1]}500C{cd[2]}500D{cd[3]}500E{cd[4]}500F{cd[5]}0" \
                  f"S15G10H10I10K10\n"
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()
        time.sleep(2.5)
        self.calibrate('111111', '8')
        # TODO: узнать, что такое blockEncPosCal
        position = [68.944, 0.0, 733.607, -90.0, 1.05, -90.0]
        # logger.debug(DEFAULT_SETTINGS['DH_r_1'])
        angles = [0, 90, -90, 0, -90, 0]
        self.jog_joints(angles)
        self.calculate_direct_kinematics_problem()

    def move_xyz(self, pos):  # mm, grad
        pos = np.array([pos])
        need_angles = self.calculate_inverse_kinematic_problem(pos)
        self.jog_joints(need_angles)
        self.calculate_direct_kinematics_problem()

    def _check_axis_limit(self, angles) -> bool:
        axis_limit = False
        for joint, angle in zip(self.joints, angles):
            if angle < joint.negative_angle_limit or angle > joint.positive_angle_limit:
                logger.error(f'{joint} AXIS LIMIT')
                axis_limit = True
        return axis_limit

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
        robot.ws_lim = self.limits
        f = robot.forward(theta)
        logger.debug(f"xyz = {f.t_3_1.reshape([3, ])}, abc = {np.degrees(f.euler_3)}")
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

    def calculate_inverse_kinematic_problem(self, x_y_z_phi_theta_psi, left=True, theta3plus=False):
        xc = x_y_z_phi_theta_psi[0]
        yc = x_y_z_phi_theta_psi[1]
        zc = x_y_z_phi_theta_psi[2]
        d1 = self.DH['d_1']
        a2 = self.DH['a_2'] # a2 и a3 по Спонгу - длины второго и третьего плеча
        a3 = self.DH['d_4']
        r = math.sqrt(xc ** 2 + yc ** 2-self.DH['a_1'])
        logger.debug(f'a2 {a2} a3 {a3}')
        s = zc - d1
        D = (r ** 2 + s ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)  # (r^2+s^2-a2^2-a3^2)/(2*a2*a3)
        # print(f"D = {D}")
        logger.debug(D)
        if(theta3plus == False):
            Theta3 = -atan2(sqrt(1 - D ** 2), D)
            Theta2 = atan2(s, r) + atan2(a3 * sin(Theta3), a2 + a3 * cos(Theta3))
        # logger.debug(Theta3)
        # Theta3 = math.acos(D)
            logger.debug(Theta3)
        else:
            Theta3 = atan2(sqrt(1 - D ** 2), D)
            Theta2 = atan2(s, r) - atan2(a3 * sin(Theta3), a2 + a3 * cos(Theta3))
        # if (D > 0 and D <= 1):
        #     Theta3 = atan2



        if (left):
            Theta1 = atan2(yc, xc)
        else:
            Theta1 = atan2(xc, yc)
        return [Theta1, Theta2, Theta3]

    def length_vector(self, point_A, point_B):
        length = sqrt((point_A[0] - point_B[0]) ** 2 + (point_A[1] - point_B[1]) ** 2 + (point_A[2] - point_B[2]) ** 2)
        return length

    def take_coordinate(self, array_matrix, number_of_matrix1, number_of_matrix2):
        # T = self.matrix_create()
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

    def inverse(self, xyz_angles, left_or_right):
        xyz_new = [xyz_angles[0] / 1000, xyz_angles[1] / 1000, xyz_angles[2] / 1000, xyz_angles[0] * pi / 180,
                   xyz_angles[1] * pi / 180, xyz_angles[2] * pi / 180]
        return self.calculate_inverse_kinematic_problem(xyz_new, left_or_right)

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
            f"x = {self.position.x} y = {self.position.y} z = {self.position.z} \n theta = {np.degrees(self.position.theta)} phi = {np.degrees(self.position.phi)} psi = {np.degrees(self.position.psi)}")
        # logger.debug(
        #  f"x_m = {self.position.x_m} y_m = {self.position.y_m} z_m = {self.position.z_m} theta_m = {np.degrees(self.position.theta_rad)} phi_m = {np.degrees(self.position.phi_rad)} psi_m = {np.degrees(self.position.psi_rad)}")
        for i in range(6):
            logger.debug(f"joint number {i + 1} have angle = {self.joints[i].current_joint_angle}")

    def show(self):
        self.calculate_direct_kinematics_problem()
        self.robot.show()

    def move_x(self, lenth_x):
        position = [self.position.x + lenth_x, self.position.y, self.position.z, self.position.theta, self.position.phi,
                    self.position.psi]
        self.move_xyz(position)

    def move_y(self, lenth_y):
        position = [self.position.x, self.position.y + lenth_y, self.position.z, self.position.theta, self.position.phi,
                    self.position.psi]
        self.move_xyz(position)

    def move_z(self, lenth_z):
        position = [self.position.x, self.position.y, self.position.z + lenth_z, self.position.theta, self.position.phi,
                    self.position.psi]
        self.move_xyz(position)

    def move_theta(self, lenth_theta):
        position = [self.position.x, self.position.y, self.position.z, self.position.theta + lenth_theta,
                    self.position.phi, self.position.psi]
        self.move_xyz(position)

    def move_phi(self, lenth_phi):
        position = [self.position.x, self.position.y, self.position.z, self.position.theta,
                    self.position.phi + lenth_phi, self.position.psi]
        self.move_xyz(position)

    def move_psi(self, lenth_psi):
        position = [self.position.x, self.position.y, self.position.z, self.position.theta, self.position.phi,
                    self.position.psi + lenth_psi]
        self.move_xyz(position)

    def grab(self):
        command = f"SV{0}P{135}\n"
        logger.debug(command)
        self.arduino_push(command)

    def absolve(self):
        command = f"SV{0}P{1}\n"
        logger.debug(command)
        self.arduino_push(command)

    def rotate_3(self, vectors, axis, angle):  # [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
        k = []
        v_norm = axis / np.linalg.norm(axis)
        R = Rotation.from_rotvec(angle * v_norm).as_matrix()
        for i in range(3):
            v = vectors[i]
            k.append(np.dot(v, R))
        return k

    def RXY_transform(self, angles):
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
        T = np.dot(rotateX, rotateY).dot(rotateZ)
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
        logger.debug(angles_xyz[3:6])
        angles = np.array([angles_xyz[3:6]])
        angles = angles.transpose()
        print(angles)
        logger.debug(angles)
        matrix = np.hstack([T, angles])
        T1 = np.vstack([matrix, [0, 0, 0, 1]])
        logger.debug(T1)

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
