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
    x: float = 0
    y: float = 0
    z: float = 0
    theta: float = 0
    phi: float = 0
    psi: float = 0

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

    def move_xyz(self, pos: Position):
        Code = 0
        print(f'pos: {pos}')
        need_angles = self.calculate_inverse_kinematic_problem([[pos.x], [pos.y], [pos.z]])
        logger.debug(f"{need_angles=}")
        need_angles = list(map(math.degrees, need_angles))
        logger.debug(f"degrees: {need_angles=}")
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
        angles = self.angular_Euler_calculation(self.matrix_dot(transform_matrix, 0, 6))  # theta, fi, psi

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
        a2 = self.DH['a_2']
        a3 = 0.22263
        r = xc**2+yc**2-d**2
        s = zc-d1
        D = (r+s**2-a2**2-a3**2)/(2*a2*a3) #(r^2+s^2-a2^2-a3^2)/(2*a2*a3)
        Theta3 = atan2(D, sqrt(1-D**2))

        Theta2 = atan2(r, s) - atan2(a2+a3*cos(Theta3), a3*sin(Theta3))

        if (left):
            Theta1 = atan2(xc, yc)
        else:
            Theta1 = atan2(xc, yc) + atan2(-sqrt(r**2-d**2), -d)
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

        Theta4 = atan2(sqrt(1-R36[3, 3]**2), R36[3, 3])
        if R36[2, 2] == abs(1):
            if Theta4 == 0:
                Theta4 == 0.001
            if Theta4 == pi:
                Theta4 = pi - 0.001
        Theta5 = atan2(R36[2, 3], R36[1, 3])
        Theta6 = atan2(R36[3, 2], R36[3, 1])

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
