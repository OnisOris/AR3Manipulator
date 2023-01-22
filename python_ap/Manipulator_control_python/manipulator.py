import time
import math
from math import (sin, cos, pi, atan2, sqrt, radians)

import numpy as np
import serial
from loguru import logger

from config import DEFAULT_SETTINGS
from joint import Joint
from dataclasses import dataclass
# from parse import parse


@dataclass(init=True)
class Position:
    """
    Class for keeping global position and orientation of manipulitor's wrist
    """
    x: float = 0  # mm
    x_m = x/1000  # m
    y: float = 0  # mm
    y_m = y/1000  # m
    z: float = 0  # mm
    z_m = z/1000  # m
    theta: float = 0  # grad
    theta_rad = theta*pi/180  # rad
    phi: float = 0  # grad
    phi_rad = phi*pi/180  # rad
    psi: float = 0  # grad
    psi_rad = psi*pi/180  # rad


    def change(self, x, y, z, theta, phi, psi):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi
        self.psi = psi


class Manipulator:
    # DH = {
    #     'a_1': 0.0642,
    #     'a_2': 0.305,
    #     'a_3': 0,
    #     'a_4': 0,
    #     'a_5': 0,
    #     'a_6': 0,
    #     'alpha_1': pi / 2,
    #     'alpha_2': 0,
    #     'alpha_3': pi / 2,
    #     'alpha_4': -pi / 2,
    #     'alpha_5': pi / 2,
    #     'alpha_6': 0,
    #     'd_1': 0.16977,
    #     'd_2': 0,
    #     'd_3': 0,
    #     'd_4': 0.22263,
    #     'd_5': 0,
    #     'd_6': 0.03625,
    #     'displacement_theta_3': pi / 2
    # }
    DH = {
        'a_1': 0.0642,
        'a_2': 0.305,
        'a_3': 0,
        'a_4': 0,
        'a_5': 0,
        'a_6': 0,
        'alpha_1': - pi / 2,
        'alpha_2': 0,
        'alpha_3': pi / 2,
        'alpha_4': -pi / 2,
        'alpha_5': pi / 2,
        'alpha_6': 0,
        'd_1': 0.16977,
        'd_2': 0,
        'd_3': 0,
        'd_4': -0.22263,
        'd_5': 0,
        'd_6': -0.03625,
        'displacement_theta_1': 0,
        'displacement_theta_2': 0,
        'displacement_theta_3': - pi / 2,
        'displacement_theta_4': 0,
        'displacement_theta_5': 0,
        'displacement_theta_6': pi
    }

    def __init__(self, teensy_port, arduino_port, baud):
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
        self.calculate_direct_kinematics_problem()

        try:
            self.serial_teensy: serial.Serial = serial.Serial(teensy_port, baud)
            self.serial_arduino: serial.Serial = serial.Serial(arduino_port, baud)
            self.is_connected = True
        except serial.SerialException:
            logger.error("Serial port not defined")

    def finish(self):
        self.serial_teensy.close()
        self.serial_arduino.close()

    def get_joints_angles(self) -> list:
        return [joint.current_joint_angle for joint in self.joints]

    def move_to(self, command):
        self.serial_teensy.write(command.encode())

    def jog_joint(self, joint: Joint, speed, degrees):  # degrees - то, на сколько градусов мы двигаем Джойнт
        # Задача направления движения джойнта и отлов ошибок
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
                joint.negative_angle_limit + (joint.current_joint_step * joint.degrees_per_step))
            self.save_data()
            # print(f"Новые координаты: {np.round(np.dot(self.calculate_direct_kinematics_problem(), 180 / pi), 3)}")
            j_jog_steps = int(round(j_jog_steps))
            command = f"MJ{joint.get_name_joint()}{drive_direction}{j_jog_steps}S{speed}G{self.ACC_dur}H{self.ACC_spd}" \
                      f"I{self.DEC_dur}K{self.DEC_spd}U{self.joints[0].current_joint_step}" \
                      f"V{self.joints[1].current_joint_step}W{self.joints[2].current_joint_step}" \
                      f"X{self.joints[3].current_joint_step}Y{self.joints[4].current_joint_step}" \
                      f"Z{self.joints[5].current_joint_step}\n"
            self.teensy_push(command)
            logger.debug(f"Write to teensy: {command.strip()}")
            self.serial_teensy.flushInput()
            time.sleep(.2)
            robot_code = str(self.serial_teensy.readline())
            # TODO: разобраться с pcode
            if robot_code[2:4] == "01":
                self.apply_robot_calibration(robot_code)

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
        self.calculate_direct_kinematics_problem()
        self.save_data()

    def teensy_push(self, command):
        self.serial_teensy.write(command.encode())

    def arduino_push(self, command):
        self.serial_arduino.write(command.encode())

    def save_data(self):
        # DEFAULT_SETTINGS['teensy_port'] = self.serial_teensy. #  TODO: разобраться со стандартными настройками здесь
        # DEFAULT_SETTINGS['arduino_port'] = self.serial_arduino.port

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
        # TODO: нужно добавить calibration_status в поле manipulitor
        if calibration_value == b'P':
            # calibration_status = 1
            for joint, cd, axis in zip(self.joints, self.calibration_direction, axes):
                if axis == '1':
                    if cd == '0':
                        joint.current_joint_step = 0
                        joint.current_joint_angle = joint.negative_angle_limit
                    else:
                        joint.current_joint_step = joint.step_limit
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
        cd = self.get_calibration_drive_auto()
        command = f"MJA{cd[0]}500B{cd[1]}500C{cd[2]}500D{cd[3]}1300E{cd[4]}500F{cd[5]}0" \
                  f"S15G10H10I10K10\n"
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command.strip()}")
        self.serial_teensy.flushInput()
        time.sleep(2.5)
        self.calibrate('111111', '8')
        # TODO: узнать, что такое blockEncPosCal
        # # blockEncPosCal = 1
        # calibration_axes = "111110"
        # speed = '40'
        # self.calibrate(calibration_axes, speed)
        # cd = self.get_calibration_drive_auto()
        # command = f"MJA{cd[0]}500B{cd[1]}500C{cd[2]}500D{cd[3]}500E{cd[4]}500F{cd[5]}0" \
        #           f"S15G10H10I10K10\n"
        # self.teensy_push(command)
        # logger.debug(f"Write to teensy: {command.strip()}")
        # self.serial_teensy.flushInput()
        #
        # speed = '8'
        # time.sleep(2.5)
        # self.calibrate(calibration_axes, speed)
        # # self.go_to_rest_position()
        #
        # calibration_axes = '000001'
        # speed = '50'
        # self.calibrate(calibration_axes, speed)
        # command = f"MJA{cd[0]}0B{cd[1]}0C{cd[2]}0D{cd[3]}0E{cd[4]}0F{cd[5]}500" \
        #           f"S15G10H10I10K10\n"
        # self.teensy_push(command)
        # logger.debug(f"Write to teensy: {command.strip()}")
        # self.serial_teensy.flushInput()
        #
        # speed = '8'
        # time.sleep(1)
        # self.calibrate(calibration_axes, speed)
        # # self.go_to_rest_position()
        # blockEncPosCal = 1

    def move_xyz(self, pos):
        Code = 0
        print(f'pos: {pos}')
        #need_angles = self.calculate_inverse_kinematic_problem([pos.x_m, pos.y_m, pos.z_m, pos.theta_rad, pos.phi_rad, pos.psi_rad], "left")
        need_angles = self.calculate_inverse_kinematics_problem2(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],'F', 0, 0, 0, 0, 0, 0)
        #need_angles = list(map(math.degrees, need_angles))

        #need_angles = [0.005, -81.87, 1.04, 13.37, 0.05, 7.17]
       # logger.debug(f"{need_angles=}")

        joint_commands = []
        joint_angels = []
        letter = 85
        if not self._check_axis_limit(need_angles):
            for i, (joint, angle) in enumerate(zip(self.joints, need_angles)):
                if float(angle) >= float(joint.current_joint_angle):
                    direction = 1 if joint.motor_direction == 0 else 0
                    calc_angle = float(angle) - float(joint.current_joint_angle)
                    steps = int(calc_angle / joint.degrees_per_step)
                    if Code != 3:
                        joint.current_joint_step += steps
                        joint.current_joint_angle = round(
                            joint.negative_angle_limit + (joint.current_joint_step * joint.degrees_per_step), 2)
                else:
                    direction = joint.motor_direction
                    calc_angle = float(joint.current_joint_angle) - float(angle)
                    steps = int(calc_angle / joint.degrees_per_step)
                    if Code != 3:
                        joint.current_joint_step -= steps
                        joint.current_joint_angle = round(
                            joint.negative_angle_limit + (joint.current_joint_step * joint.degrees_per_step), 2)
                joint_commands.append(f"{joint.get_name_joint()}{direction}{steps}")
                joint_angels.append(f"{chr(letter + i)}{joint.current_joint_step}")

            commandCalc = f"MJ{''.join(joint_commands)}S{30}G{15}H{10}I{20}K{5}\n"
            logger.debug(commandCalc.strip())
            self.teensy_push(commandCalc)
        self.calculate_direct_kinematics_problem()
        self.save_data()

    def _check_axis_limit(self, angles) -> bool:
        axis_limit = False
        for joint, angle in zip(self.joints, angles):
            if angle < joint.negative_angle_limit or angle > joint.positive_angle_limit:
                logger.error(f'{joint} AXIS LIMIT')
                axis_limit = True
        return axis_limit

    def calculate_direct_kinematics_problem(self) -> None:
        for joint in self.joints:
            if joint.get_current_joint_angle() == 0:
                joint.current_joint_angle = 0.000000000001  # TODO: разобраться с необходимостью данных операций upd. Влияет на знак в углах эйлера, пока не понятно как
        transform_matrix = self.matrix_create()
        p = self.take_coordinate(transform_matrix, 0, 6)
        p = [p[0]*1000, p[1]*1000, p[2]*1000] # перевод
        angles = self.angular_Euler_calculation(self.matrix_dot(transform_matrix, 0, 6))  # theta, fi, psi
        angles = [angles[0]/pi*180, angles[1]/pi*180, angles[2]/pi*180]

        self.position.change(*list(p), *angles)

    def matrix_create(self):
        cja = [float(self.joints[0].current_joint_angle), float(self.joints[1].current_joint_angle),
               float(self.joints[2].current_joint_angle),
               float(self.joints[3].current_joint_angle), float(self.joints[4].current_joint_angle),
               float(self.joints[5].current_joint_angle)]
        cja = list(map(radians, cja))
        T = []
        # displacement_theta_3 = self.DH['displacement_theta_3']
        for i in range(6):
            # d = 0
            # if i == 2:
            #     d = displacement_theta_3
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
        # global theta, fi, psi
        rotation_matrix = transform_matrix0_6[0:3, 0:3]
        r3_3 = transform_matrix0_6[2, 2]
        r2_3 = transform_matrix0_6[1, 2]
        r1_3 = transform_matrix0_6[0, 2]
        r3_2 = transform_matrix0_6[2, 1]
        r3_1 = transform_matrix0_6[2, 0]
        r1_1 = transform_matrix0_6[0, 0]
        r2_1 = transform_matrix0_6[1, 0]
        r1_2 = transform_matrix0_6[0, 1]
        if r3_3 != 1 or -1:
            theta = atan2(sqrt(1 - r3_3 ** 2), r3_3)
            fi = atan2(r2_3, r1_3)
            psi = atan2(r3_2, -r3_1)
        if r3_3 == 1:
            theta = 0
            fi = atan2(r2_1, r1_1)
            psi = 0
        if r3_3 == -1:
            theta = pi
            fi = atan2(-r1_2, -r1_1)
            psi = 0

        return [theta, fi, psi]  # углы Эйлера схвата в главной системе координат TODO: точно такой порядок углов???

    def calculate_inverse_kinematic_problem(self, x_y_z_phi_theta_psi, left):
        self.anti_zero()
        # Теперь делаем все по методе Спонга
        xc = x_y_z_phi_theta_psi[0]
        yc = x_y_z_phi_theta_psi[1]
        zc = x_y_z_phi_theta_psi[2]
        d = 0.0642
        d1 = 0.16977
        a2 = 0.305
        a3 = 0.22263
        r = xc ** 2 + yc ** 2 - d ** 2
        logger.debug(r)
        s = zc - d1
        D = (r ** 2 + s ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)  # (r^2+s^2-a2^2-a3^2)/(2*a2*a3)
        print(f"D = {D}")
        # print(D)
        Theta3 = atan2(D, sqrt(1 - D ** 2))

        Theta2 = atan2(r, s) - atan2(a2 + a3 * cos(Theta3), a3 * sin(Theta3))

        if (left):
            Theta1 = atan2(xc, yc)
        else:
            Theta1 = atan2(xc, yc) + atan2(-sqrt(r ** 2 - d ** 2), -d)
        cja = [Theta1, Theta2, Theta3]
        cja = list(map(radians, cja))
        T = []
        for i in range(3):
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
        R03 = self.take_rotation_matrix(T, 0, 3)
        # Углы эйлера вокруг осей x, y, z TODO: проверить, что углы соответствуют осям
        phi = x_y_z_phi_theta_psi[3]
        theta = x_y_z_phi_theta_psi[4]
        psi = x_y_z_phi_theta_psi[5]
        # Преобразуем в матрицу поворота

        # Матрицы поворота вокруг осей
        Rz1 = np.array([[cos(phi), -sin(phi), 0],
                        [sin(phi), cos(phi), 0],
                        [0, 0, 1]])
        Ry = np.array([[cos(theta), 0, sin(theta)],
                       [0, 1, 0],
                       [-sin(theta), 0, cos(theta)]])
        Rz2 = np.array([[cos(psi), -sin(psi), 0],
                        [sin(psi), cos(psi), 0],
                        [0, 0, 1]])
        # Матрица преобразования, параметризованная углами Эйлера
        Rxyz = np.dot(np.dot(Rz1, Ry), Rz2)

        # R36 = (R03)^T*Rxyz

        R36 = np.dot(np.transpose(R03), Rxyz)
        logger.debug(R36)
        Theta4 = atan2(sqrt(1 - R36[2, 2] ** 2), R36[2, 2])
        if R36[2, 2] == abs(1):
            if Theta4 == 0:
                Theta4 == 0.001
            if Theta4 == pi:
                Theta4 = pi - 0.001
        Theta5 = atan2(R36[1, 2], R36[0, 2])
        Theta6 = atan2(R36[2, 1], R36[2, 0])

        return [Theta1, Theta2, Theta3, Theta4, Theta5, Theta6]

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
        xyz_new = [xyz_angles[0]/1000, xyz_angles[1]/1000, xyz_angles[2]/1000, xyz_angles[0]*pi/180, xyz_angles[1]*pi/180, xyz_angles[2]*pi/180]
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

    def calculate_inverse_kinematics_problem2(self, CX, CY, CZ, CRx, CRy, CRz, WC, TCX, TCY, TCZ, TCRx, TCRy, TCRz):
        # global J1out
        # global J2out
        # global J3out
        # global J4out
        # global J5out

        # global J6out
        J1AngCur = self.joints[0]
        J2AngCur = self.joints[1]
        J3AngCur = self.joints[2]
        J4AngCur = self.joints[3]
        J5AngCur = self.joints[4]
        J6AngCur = self.joints[5]
        if J1AngCur == 0:
            J1AngCur = .0001
        if J2AngCur == 0:
            J2AngCur = .0001
        if J3AngCur == 0:
            J3AngCur = .0001
        if J4AngCur == 0:
            J4AngCur = .0001
        if J5AngCur == 0:
            J5AngCur = .0001
        if J6AngCur == 0:
            J6AngCur = .0001
            # input
        O4 = CX
        O5 = CY
        O6 = CZ
        O9 = CRx
        O8 = CRy
        O7 = CRz
        V8 = WC
        if O4 == 0:
            O4 = .0001
        if O5 == 0:
            O5 = .0001
        if O6 == 0:
            O6 = .0001
        if O7 == 0:
            O7 = .0001
        if O8 == 0:
            O8 = .0001
        if O9 == 0:
            O9 = .0001
        # quadrant
        if O4 > 0 and O5 > 0:
            V9 = 1
        elif O4 > 0 and O5 < 0:
            V9 = 2
        elif O4 < 0 and O5 < 0:
            V9 = 3
        elif O4 < 0 and O5 > 0:
            V9 = 4
        # DH TABLE
        D13 = math.radians(-90)
        D14 = math.radians(0.0)
        D15 = math.radians(90)
        D16 = math.radians(-90)
        D17 = math.radians(90)
        D18 = math.radians(0)

        E13 = 169.77
        E14 = 0.0
        E15 = 0.0
        E16 = -222.63
        E17 = 0.0
        E18 = -36.25

        F13 = 64.2
        F14 = 305.0
        F15 = 0.0
        F16 = 0.0
        F17 = 0.0
        F18 = 0.0
        # WORK FRAME INPUT
        H13 = -float(0)
        H14 = -float(0)
        H15 = -float(0)
        H16 = -float(0)
        H17 = -float(0)
        H18 = -float(0)
        # TOOL FRAME INPUT
        J13 = -float(0) + TCX
        J14 = -float(0) + TCY
        J15 = -float(0) + TCZ
        J16 = -float(0) + TCRx
        J17 = -float(0) + TCRy
        J18 = -float(0) + TCRz
        # WORK FRAME TABLE
        N30 = math.cos(math.radians(H18)) * math.cos(math.radians(H17))
        O30 = -math.sin(math.radians(H18)) * math.cos(math.radians(H16)) + math.cos(math.radians(H18)) * math.sin(
            math.radians(H17)) * math.sin(math.radians(H16))
        P30 = math.sin(math.radians(H18)) * math.sin(math.radians(H16)) + math.cos(math.radians(H18)) * math.sin(
            math.radians(H17)) * math.cos(math.radians(H16))
        Q30 = H13
        N31 = math.sin(math.radians(H18)) * math.cos(math.radians(H17))
        O31 = math.cos(math.radians(H18)) * math.cos(math.radians(H16)) + math.sin(math.radians(H18)) * math.sin(
            math.radians(H17)) * math.sin(math.radians(H16))
        P31 = -math.cos(math.radians(H18)) * math.sin(math.radians(H16)) + math.sin(math.radians(H18)) * math.sin(
            math.radians(H17)) * math.cos(math.radians(H16))
        Q31 = H14
        N32 = -math.sin(math.radians(H18))
        O32 = math.cos(math.radians(H17)) * math.sin(math.radians(H16))
        P32 = math.cos(math.radians(H17)) * math.cos(math.radians(H16))
        Q32 = H15
        N33 = 0
        O33 = 0
        P33 = 0
        Q33 = 1
        # R 0-T
        X30 = math.cos(math.radians(O7)) * math.cos(math.radians(O9)) - math.cos(math.radians(O8)) * math.sin(
            math.radians(O7)) * math.sin(math.radians(O9))
        Y30 = math.cos(math.radians(O9)) * math.sin(math.radians(O7)) + math.cos(math.radians(O7)) * math.cos(
            math.radians(O8)) * math.sin(math.radians(O9))
        Z30 = math.sin(math.radians(O8)) * math.sin(math.radians(O9))
        AA30 = O4
        X31 = math.cos(math.radians(O8)) * math.cos(math.radians(O9)) * math.sin(math.radians(O7)) + math.cos(
            math.radians(O7)) * math.sin(math.radians(O9))
        Y31 = math.cos(math.radians(O7)) * math.cos(math.radians(O8)) * math.cos(math.radians(O9)) - math.sin(
            math.radians(O7)) * math.sin(math.radians(O9))
        Z31 = math.cos(math.radians(O9)) * math.sin(math.radians(O8))
        AA31 = O5
        X32 = math.sin(math.radians(O7)) * math.sin(math.radians(O8))
        Y32 = math.cos(math.radians(O7)) * math.sin(math.radians(O8))
        Z32 = -math.cos(math.radians(O8))
        AA32 = O6
        X33 = 0
        Y33 = 0
        Z33 = 0
        AA33 = 1
        # R 0-T   offset by work frame
        X36 = ((N30 * X30) + (O30 * X31) + (P30 * X32) + (Q30 * X33)) * -1
        Y36 = (N30 * Y30) + (O30 * Y31) + (P30 * Y32) + (Q30 * Y33)
        Z36 = (N30 * Z30) + (O30 * Z31) + (P30 * Z32) + (Q30 * Z33)
        AA36 = (N30 * AA30) + (O30 * AA31) + (P30 * AA32) + (Q30 * AA33)
        X37 = (N31 * X30) + (O31 * X31) + (P31 * X32) + (Q31 * X33)
        Y37 = (N31 * Y30) + (O31 * Y31) + (P31 * Y32) + (Q31 * Y33)
        Z37 = (N31 * Z30) + (O31 * Z31) + (P31 * Z32) + (Q31 * Z33)
        AA37 = (N31 * AA30) + (O31 * AA31) + (P31 * AA32) + (Q31 * AA33)
        X38 = (N32 * X30) + (O32 * X31) + (P32 * X32) + (Q32 * X33)
        Y38 = (N32 * Y30) + (O32 * Y31) + (P32 * Y32) + (Q32 * Y33)
        Z38 = (N32 * Z30) + (O32 * Z31) + (P32 * Z32) + (Q32 * Z33)
        AA38 = (N32 * AA30) + (O32 * AA31) + (P32 * AA32) + (Q32 * AA33)
        X39 = (N33 * X30) + (O33 * X31) + (P33 * X32) + (Q33 * X33)
        Y39 = (N33 * Y30) + (O33 * Y31) + (P33 * Y32) + (Q33 * Y33)
        Z39 = (N33 * Z30) + (O33 * Z31) + (P33 * Z32) + (Q33 * Z33)
        AA39 = (N33 * AA30) + (O33 * AA31) + (P33 * AA32) + (Q33 * AA33)
        # TOOL FRAME
        X42 = math.cos(math.radians(J18)) * math.cos(math.radians(J17))
        Y42 = -math.sin(math.radians(J18)) * math.cos(math.radians(J16)) + math.cos(math.radians(J18)) * math.sin(
            math.radians(J17)) * math.sin(math.radians(J16))
        Z42 = math.sin(math.radians(J18)) * math.sin(math.radians(J16)) + math.cos(math.radians(J18)) * math.sin(
            math.radians(J17)) * math.cos(math.radians(J16))
        AA42 = J13
        X43 = math.sin(math.radians(J18)) * math.cos(math.radians(J17))
        Y43 = math.cos(math.radians(J18)) * math.cos(math.radians(J16)) + math.sin(math.radians(J18)) * math.sin(
            math.radians(J17)) * math.sin(math.radians(J16))
        Z43 = -math.cos(math.radians(J18)) * math.sin(math.radians(J16)) + math.sin(math.radians(J18)) * math.sin(
            math.radians(J17)) * math.cos(math.radians(J16))
        AA43 = J14
        X44 = -math.sin(math.radians(J18))
        Y44 = math.cos(math.radians(J17)) * math.sin(math.radians(J16))
        Z44 = math.cos(math.radians(J17)) * math.cos(math.radians(J16))
        AA44 = J15
        X45 = 0
        Y45 = 0
        Z45 = 0
        AA45 = 1
        # INVERT TOOL FRAME
        X48 = X42
        Y48 = X43
        Z48 = X44
        AA48 = (X48 * AA42) + (Y48 * AA43) + (Z48 * AA44)
        X49 = Y42
        Y49 = Y43
        Z49 = Y44
        AA49 = (X49 * AA42) + (Y49 * AA43) + (Z49 * AA44)
        X50 = Z42
        Y50 = Z43
        Z50 = Z44
        AA50 = (X50 * AA42) + (Y50 * AA43) + (Z50 * AA44)
        X51 = 0
        Y51 = 0
        Z51 = 0
        AA51 = 1
        # R 0-6
        X54 = (X36 * X48) + (Y36 * X49) + (Z36 * X50) + (AA36 * X51)
        Y54 = (X36 * Y48) + (Y36 * Y49) + (Z36 * Y50) + (AA36 * Y51)
        Z54 = (X36 * Z48) + (Y36 * Z49) + (Z36 * Z50) + (AA36 * Z51)
        AA54 = (X36 * AA48) + (Y36 * AA49) + (Z36 * AA50) + (AA36 * AA51)
        X55 = (X37 * X48) + (Y37 * X49) + (Z37 * X50) + (AA37 * X51)
        Y55 = (X37 * Y48) + (Y37 * Y49) + (Z37 * Y50) + (AA37 * Y51)
        Z55 = (X37 * Z48) + (Y37 * Z49) + (Z37 * Z50) + (AA37 * Z51)
        AA55 = (X37 * AA48) + (Y37 * AA49) + (Z37 * AA50) + (AA37 * AA51)
        X56 = (X38 * X48) + (Y38 * X49) + (Z38 * X50) + (AA38 * X51)
        Y56 = (X38 * Y48) + (Y38 * Y49) + (Z38 * Y50) + (AA38 * Y51)
        Z56 = (X38 * Z48) + (Y38 * Z49) + (Z38 * Z50) + (AA38 * Z51)
        AA56 = (X38 * AA48) + (Y38 * AA49) + (Z38 * AA50) + (AA38 * AA51)
        X57 = (X39 * X48) + (Y39 * X49) + (Z39 * X50) + (AA39 * X51)
        Y57 = (X39 * Y48) + (Y39 * Y49) + (Z39 * Y50) + (AA39 * Y51)
        Z57 = (X39 * Z48) + (Y39 * Z49) + (Z39 * Z50) + (AA39 * Z51)
        AA57 = (X39 * AA48) + (Y39 * AA49) + (Z39 * AA50) + (AA39 * AA51)
        # REMOVE R 0-6
        X60 = math.cos(math.radians(180))
        Y60 = math.sin(math.radians(180))
        Z60 = 0
        AA60 = 0
        X61 = -math.sin(math.radians(180)) * math.cos(D18)
        Y61 = math.cos(math.radians(180)) * math.cos(D18)
        Z61 = math.sin(D18)
        AA61 = 0
        X62 = math.sin(math.radians(180)) * math.sin(D18)
        Y62 = -math.cos(math.radians(180)) * math.sin(D18)
        Z62 = math.cos(D18)
        AA62 = -E18
        X63 = 0
        Y63 = 0
        Z63 = 0
        AA63 = 1
        # R 0-5 (center spherica wrist)
        X66 = (X54 * X60) + (Y54 * X61) + (Z54 * X62) + (AA54 * X63)
        Y66 = (X54 * Y60) + (Y54 * Y61) + (Z54 * Y62) + (AA54 * Y63)
        Z66 = (X54 * Z60) + (Y54 * Z61) + (Z54 * Z62) + (AA54 * Z63)
        AA66 = (X54 * AA60) + (Y54 * AA61) + (Z54 * AA62) + (AA54 * AA63)
        X67 = (X55 * X60) + (Y55 * X61) + (Z55 * X62) + (AA55 * X63)
        Y67 = (X55 * Y60) + (Y55 * Y61) + (Z55 * Y62) + (AA55 * Y63)
        Z67 = (X55 * Z60) + (Y55 * Z61) + (Z55 * Z62) + (AA55 * Z63)
        AA67 = (X55 * AA60) + (Y55 * AA61) + (Z55 * AA62) + (AA55 * AA63)
        X68 = (X56 * X60) + (Y56 * X61) + (Z56 * X62) + (AA56 * X63)
        Y68 = (X56 * Y60) + (Y56 * Y61) + (Z56 * Y62) + (AA56 * Y63)
        Z68 = (X56 * Z60) + (Y56 * Z61) + (Z56 * Z62) + (AA56 * Z63)
        AA68 = (X56 * AA60) + (Y56 * AA61) + (Z56 * AA62) + (AA56 * AA63)
        X69 = (X57 * X60) + (Y57 * X61) + (Z57 * X62) + (AA57 * X63)
        Y69 = (X57 * Y60) + (Y57 * Y61) + (Z57 * Y62) + (AA57 * Y63)
        Z69 = (X57 * Z60) + (Y57 * Z61) + (Z57 * Z62) + (AA57 * Z63)
        AA69 = (X57 * AA60) + (Y57 * AA61) + (Z57 * AA62) + (AA57 * AA63)
        # CALCULATE J1 ANGLE
        O13 = math.atan(AA67 / AA66)
        if V9 == 1:
            P13 = math.degrees(O13)
        if V9 == 2:
            P13 = math.degrees(O13)
        if V9 == 3:
            P13 = -180 + math.degrees(O13)
        if V9 == 4:
            P13 = 180 + math.degrees(O13)
        # CALCULATE J2 ANGLE	FWD

        O16 = math.sqrt(((abs(AA67)) ** 2) + ((abs(AA66)) ** 2))
        O17 = AA68 - E13
        O18 = O16 - F13
        O19 = math.sqrt((O17 ** 2) + (O18 ** 2))
        O20 = math.sqrt((E16 ** 2) + (F15 ** 2))
        O21 = math.degrees(math.atan(O17 / O18))
        O22 = math.degrees(math.acos(((F14 ** 2) + (O19 ** 2) - (abs(O20) ** 2)) / (2 * F14 * O19)))
        try:
            O25 = math.degrees(math.atan(abs(E16) / F15))
        except:
            O25 = 90
        O23 = 180 - math.degrees(math.acos(((abs(O20) ** 2) + (F14 ** 2) - (O19 ** 2)) / (2 * abs(O20) * F14))) + (
                90 - O25)
        O26 = -(O21 + O22)
        O27 = O23
        # CALCULATE J2 ANGLE	MID
        P18 = -O18
        P19 = math.sqrt((O17 ** 2) + (P18 ** 2))
        P21 = math.degrees(math.acos(((F14 ** 2) + (P19 ** 2) - (abs(O20) ** 2)) / (2 * F14 * P19)))
        P22 = math.degrees(math.atan(P18 / O17))
        P23 = 180 - math.degrees(math.acos(((abs(O20) ** 2) + (F14 ** 2) - (P19 ** 2)) / (2 * abs(O20) * F14))) + (
                90 - O25)
        P24 = 90 - (P21 + P22)
        P26 = -180 + P24
        P27 = P23
        # J1,J2,J3
        Q4 = P13
        if O18 < 0:
            Q5 = P26
            Q6 = P27
        else:
            Q5 = O26
            Q6 = O27
        # J1
        N36 = math.cos(math.radians(Q4))
        O36 = -math.sin(math.radians(Q4)) * math.cos(D13)
        P36 = math.sin(math.radians(Q4)) * math.sin(D13)
        Q36 = F13 * math.cos(math.radians(Q4))
        N37 = math.sin(math.radians(Q4))
        O37 = math.cos(math.radians(Q4)) * math.cos(D13)
        P37 = -math.cos(math.radians(Q4)) * math.sin(D13)
        Q37 = F13 * math.sin(math.radians(Q4))
        N38 = 0
        O38 = math.sin(D13)
        P38 = math.cos(D13)
        Q38 = E13
        N39 = 0
        O39 = 0
        P39 = 0
        Q39 = 1
        # J2
        N42 = math.cos(math.radians(Q5))
        O42 = -math.sin(math.radians(Q5)) * math.cos(D14)
        P42 = math.sin(math.radians(Q5)) * math.sin(D14)
        Q42 = F14 * math.cos(math.radians(Q5))
        N43 = math.sin(math.radians(Q5))
        O43 = math.cos(math.radians(Q5)) * math.cos(D14)
        P43 = -math.cos(math.radians(Q5)) * math.sin(D14)
        Q43 = F14 * math.sin(math.radians(Q5))
        N44 = 0
        O44 = math.sin(D14)
        P44 = math.cos(D14)
        Q44 = E14
        N45 = 0
        O45 = 0
        P45 = 0
        Q45 = 1
        # J3
        N48 = math.cos(math.radians(Q6 - 90))
        O48 = -math.sin(math.radians(Q6 - 90)) * math.cos(D15)
        P48 = math.sin(math.radians(Q6 - 90)) * math.sin(D15)
        Q48 = F15 * math.cos(math.radians(Q6 - 90))
        N49 = math.sin(math.radians(Q6 - 90))
        O49 = math.cos(math.radians(Q6 - 90)) * math.cos(D15)
        P49 = -math.cos(math.radians(Q6 - 90)) * math.sin(D15)
        Q49 = F15 * math.sin(math.radians(Q6 - 90))
        N50 = 0
        O50 = math.sin(D15)
        P50 = math.cos(D15)
        Q50 = E15
        N51 = 0
        O51 = 0
        P51 = 0
        Q51 = 0
        # R 0-1
        S33 = (N30 * N36) + (O30 * N37) + (P30 * N38) + (Q30 * N39)
        T33 = (N30 * O36) + (O30 * O37) + (P30 * O38) + (Q30 * O39)
        U33 = (N30 * P36) + (O30 * P37) + (P30 * P38) + (Q30 * P39)
        V33 = (N30 * Q36) + (O30 * Q37) + (P30 * Q38) + (Q30 * Q39)
        S34 = (N31 * N36) + (O31 * N37) + (P31 * N38) + (Q31 * N39)
        T34 = (N31 * O36) + (O31 * O37) + (P31 * O38) + (Q31 * O39)
        U34 = (N31 * P36) + (O31 * P37) + (P31 * P38) + (Q31 * P39)
        V34 = (N31 * Q36) + (O31 * Q37) + (P31 * Q38) + (Q31 * Q39)
        S35 = (N32 * N36) + (O32 * N37) + (P32 * N38) + (Q32 * N39)
        T35 = (N32 * O36) + (O32 * O37) + (P32 * O38) + (Q32 * O39)
        U35 = (N32 * P36) + (O32 * P37) + (P32 * P38) + (Q32 * P39)
        V35 = (N32 * Q36) + (O32 * Q37) + (P32 * Q38) + (Q32 * Q39)
        S36 = (N33 * N36) + (O33 * N37) + (P33 * N38) + (Q33 * N39)
        T36 = (N33 * O36) + (O33 * O37) + (P33 * O38) + (Q33 * O39)
        U36 = (N33 * P36) + (O33 * P37) + (P33 * P38) + (Q33 * P39)
        V36 = (N33 * Q36) + (O33 * Q37) + (P33 * Q38) + (Q33 * Q39)
        # R 0-2
        S39 = (S33 * N42) + (T33 * N43) + (U33 * N44) + (V33 * N45)
        T39 = (S33 * O42) + (T33 * O43) + (U33 * O44) + (V33 * O45)
        U39 = (S33 * P42) + (T33 * P43) + (U33 * P44) + (V33 * P45)
        V39 = (S33 * Q42) + (T33 * Q43) + (U33 * Q44) + (V33 * Q45)
        S40 = (S34 * N42) + (T34 * N43) + (U34 * N44) + (V34 * N45)
        T40 = (S34 * O42) + (T34 * O43) + (U34 * O44) + (V34 * O45)
        U40 = (S34 * P42) + (T34 * P43) + (U34 * P44) + (V34 * P45)
        V40 = (S34 * Q42) + (T34 * Q43) + (U34 * Q44) + (V34 * Q45)
        S41 = (S35 * N42) + (T35 * N43) + (U35 * N44) + (V35 * N45)
        T41 = (S35 * O42) + (T35 * O43) + (U35 * O44) + (V35 * O45)
        U41 = (S35 * P42) + (T35 * P43) + (U35 * P44) + (V35 * P45)
        V41 = (S35 * Q42) + (T35 * Q43) + (U35 * Q44) + (V35 * Q45)
        S42 = (S36 * N42) + (T36 * N43) + (U36 * N44) + (V36 * N45)
        T42 = (S36 * O42) + (T36 * O43) + (U36 * O44) + (V36 * O45)
        U42 = (S36 * P42) + (T36 * P43) + (U36 * P44) + (V36 * P45)
        V42 = (S36 * Q42) + (T36 * Q43) + (U36 * Q44) + (V36 * Q45)
        # R 0-3
        S45 = (S39 * N48) + (T39 * N49) + (U39 * N50) + (V39 * N51)
        T45 = (S39 * O48) + (T39 * O49) + (U39 * O50) + (V39 * O51)
        U45 = (S39 * P48) + (T39 * P49) + (U39 * P50) + (V39 * P51)
        V45 = (S39 * Q48) + (T39 * Q49) + (U39 * Q50) + (V39 * Q51)
        S46 = (S40 * N48) + (T40 * N49) + (U40 * N50) + (V40 * N51)
        T46 = (S40 * O48) + (T40 * O49) + (U40 * O50) + (V40 * O51)
        U46 = (S40 * P48) + (T40 * P49) + (U40 * P50) + (V40 * P51)
        V46 = (S40 * Q48) + (T40 * Q49) + (U40 * Q50) + (V40 * Q51)
        S47 = (S41 * N48) + (T41 * N49) + (U41 * N50) + (V41 * N51)
        T47 = (S41 * O48) + (T41 * O49) + (U41 * O50) + (V41 * O51)
        U47 = (S41 * P48) + (T41 * P49) + (U41 * P50) + (V41 * P51)
        V47 = (S41 * Q48) + (T41 * Q49) + (U41 * Q50) + (V41 * Q51)
        S48 = (S42 * N48) + (T42 * N49) + (U42 * N50) + (V42 * N51)
        T48 = (S42 * O48) + (T42 * O49) + (U42 * O50) + (V42 * O51)
        U48 = (S42 * P48) + (T42 * P49) + (U42 * P50) + (V42 * P51)
        V48 = (S42 * Q48) + (T42 * Q49) + (U42 * Q50) + (V42 * Q51)
        # R 0-3 transposed
        S51 = S45
        T51 = S46
        U51 = S47
        S52 = T45
        T52 = T46
        U52 = T47
        S53 = U45
        T53 = U46
        U53 = U47
        # R 3-6 (spherical wrist  orientation)
        X72 = (S51 * X66) + (T51 * X67) + (U51 * X68)
        Y72 = (S51 * Y66) + (T51 * Y67) + (U51 * Y68)
        Z72 = (S51 * Z66) + (T51 * Z67) + (U51 * Z68)
        X73 = (S52 * X66) + (T52 * X67) + (U52 * X68)
        Y73 = (S52 * Y66) + (T52 * Y67) + (U52 * Y68)
        Z73 = (S52 * Z66) + (T52 * Z67) + (U52 * Z68)
        X74 = (S53 * X66) + (T53 * X67) + (U53 * X68)
        Y74 = (S53 * Y66) + (T53 * Y67) + (U53 * Y68)
        Z74 = (S53 * Z66) + (T53 * Z67) + (U53 * Z68)
        # WRIST ORIENTATION
        R7 = math.degrees(math.atan2(Z73, Z72))
        R8 = math.degrees(math.atan2(+math.sqrt(1 - Z74 ** 2), Z74))
        if Y74 < 0:
            R9 = math.degrees(math.atan2(-Y74, X74)) - 180
        else:
            R9 = math.degrees(math.atan2(-Y74, X74)) + 180
        S7 = math.degrees(math.atan2(-Z73, -Z72))
        S8 = math.degrees(math.atan2(-math.sqrt(1 - Z74 ** 2), Z74))
        if Y74 < 0:
            S9 = math.degrees(math.atan2(Y74, -X74)) + 180
        else:
            S9 = math.degrees(math.atan2(Y74, -X74)) - 180
        if V8 == "F":
            Q8 = R8
        else:
            Q8 = S8
        if Q8 > 0:
            Q7 = R7
        else:
            Q7 = S7
        if Q8 < 0:
            Q9 = S9
        else:
            Q9 = R9
        # FINAL OUTPUT
        J1out = Q4
        J2out = Q5
        J3out = Q6
        J4out = Q7
        J5out = Q8
        J6out = Q9
        return J1out, J2out, J3out, J4out, J5out, J6out
