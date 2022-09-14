import time
from math import (sin, cos, pi, atan2, sqrt)

import numpy as np
import serial
from loguru import logger

from config import DEFAULT_SETTINGS
from joint import Joint


class Manipulator:
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

        try:
            self.serial_teensy: serial.Serial = serial.Serial(teensy_port, baud)
            self.serial_arduino: serial.Serial = serial.Serial(arduino_port, baud)
            self.is_connected = True
        except serial.SerialException:
            logger.error("Serial port not defined")

    def move_to(self, command):
        self.serial_teensy.write(command.encode())

    def jog_joint(self, joint: Joint, speed, degrees):
        if not self.JogStepsStat:  # JogStepsStat показывает, в каких единицах мы будем передвигать джойнт, либо в шагах, либо в градусах
            j_jog_steps = int(
                degrees / joint.degrees_per_step)  # высчитываем количенство шагов, joint.degrees_per_step -
        else:
            # switch from degs to steps
            j_jog_steps = degrees
            degrees *= joint.degrees_per_step
        drive_direction = 1 if joint.motor_direction == 0 else 0  #
        if degrees <= (joint.positive_angle_limit - joint.current_joint_angle):
            joint.current_joint_step += int(j_jog_steps)
            joint.current_joint_angle = round(
                joint.negative_angle_limit + (joint.current_joint_step * joint.degrees_per_step))
            self.save_data()  # TODO:ДОдлЕАТЬ save_position_data() и calculate_direct_kinematics_problem()
            # calculate_direct_kinematics_problem()
            command = f"MJ{joint.get_name_joint()}{drive_direction}{j_jog_steps}S{speed}G{self.ACC_dur}H{self.ACC_spd}" \
                      f"I{self.DEC_dur}K{self.DEC_spd}U{self.joints[0].current_joint_step}" \
                      f"V{self.joints[1].current_joint_step}W{self.joints[2].current_joint_step}" \
                      f"X{self.joints[3].current_joint_step}Y{self.joints[4].current_joint_step}" \
                      f"Z{self.joints[5].current_joint_step}\n"
            self.teensy_push(command)
            logger.debug(f"Write to teensy: {command}")
            self.serial_teensy.flushInput()
            time.sleep(.2)
            robot_code = str(self.serial_teensy.readline())
            logger.info(robot_code)
            # TODO: разобраться с pcode
            if robot_code[2:4] == "01":
                self.apply_robot_calibration(robot_code)
        else:
            logger.warning(f"Joint {joint.number_joint} AXIS LIMIT")

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
                logger.error(f'{joint.get_name_joint()} COLLISION OR OUT OF CALIBRATION')

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
        DEFAULT_SETTINGS['teensy_port'] = self.serial_teensy.port
        DEFAULT_SETTINGS['arduino_port'] = self.serial_arduino.port

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
        logger.debug(f"Write to teensy: {command}")

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
            logger.warning('NO CAL FEEDBACK FROM ARDUINO')

        self.calculate_direct_kinematics_problem()
        self.save_data()
        joints_current_steps = [f"{joint.get_name_joint()}{joint.current_joint_step}" for joint in self.joints]
        command = f'LM{"".join(joints_current_steps)}\n'
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command}")
        self.serial_teensy.flushInput()

    def auto_calibrate(self):
        # TODO: узнать, что такое blockEncPosCal
        # blockEncPosCal = 1
        calibration_axes = "111110"
        speed = '50'
        self.calibrate(calibration_axes, speed)
        cd = self.get_calibration_drive()
        command = f"MJA{cd[0]}500B{cd[1]}500C{cd[2]}500D{cd[3]}500E{cd[4]}500F{cd[5]}0" \
                  f"S15G10H10I10K10\n"
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command}")
        self.serial_teensy.flushInput()

        speed = '8'
        time.sleep(2.5)
        self.calibrate(calibration_axes, speed)
        # gotoRestPos()

        calibration_axes = '000001'
        speed = '50'
        self.calibrate(calibration_axes, speed)
        command = f"MJA{cd[0]}0B{cd[1]}0C{cd[2]}0D{cd[3]}0E{cd[4]}0F{cd[5]}500" \
                  f"S15G10H10I10K10\n"
        self.teensy_push(command)
        logger.debug(f"Write to teensy: {command}")
        self.serial_teensy.flushInput()

        speed = '8'
        time.sleep(1)
        self.calibrate(calibration_axes, speed)
        # gotoRestPos()
        logger.success('CALIBRATION SUCCESSFUL')
        # blockEncPosCal = 1

    def calculate_direct_kinematics_problem(self):
        # for joint in self.joints:
        #     if joint.get_current_joint_angle() == 0:
        #         joint.current_joint_angle = 0.0001
        transform_matrix = self.matrix_create()
        # eye = np.eye(4)
        # print(eye)
        # print(transform_matrix)
        # print(self.angular_Euler_calculation(transform_matrix))
        self.calculate_inverse_kinematics_problem(transform_matrix)
        return transform_matrix

    def matrix_create(self):
        cja = [float(self.joints[0].current_joint_angle), float(self.joints[1].current_joint_angle),
               float(self.joints[2].current_joint_angle),
               float(self.joints[3].current_joint_angle), float(self.joints[4].current_joint_angle),
               float(self.joints[5].current_joint_angle)]
        TS = {'a_1': 64.20, 'a_2': 0, 'a_3': 0, 'a_4': 0, 'a_5': 0, 'a_6': 0,
              'alpha_1': pi / 2, 'alpha_2': 0, 'alpha_3': pi / 2, 'alpha_4': -pi / 2,
              'alpha_5': pi / 2, 'alpha_6': 0,
              'd_1': 169.77, 'd_2': 0, 'd_3': 0, 'd_4': 222.63, 'd_5': 0, 'd_6': 36.25,
              'displacement_theta_3': pi / 2}
        T = []
        for i in range(6):
            T.append(np.array(
                [[cos(cja[i]), -sin(cja[i]) * cos(TS[f'alpha_{i + 1}']), sin(cja[i]) * sin(TS[f'alpha_{i + 1}']),
                  TS[f'a_{i + 1}'] * cos(cja[i])],
                 [sin(cja[i]), cos(cja[i]) * cos(TS[f'alpha_{i + 1}']), -cos(cja[i]) * sin(TS[f'alpha_{i + 1}']),
                  TS[f'a_{i + 1}'] * sin(cja[i])],
                 [0, sin(cja[i]), cos(cja[i]), TS[f'd_{i + 1}']],
                 [0, 0, 0, 1]]))
            # print(T[i])
        return T

    def matrix_dot_all(self, array_matrix):
        T0_6 = np.dot(array_matrix[5], np.dot(array_matrix[4], np.dot(array_matrix[3], np.dot(array_matrix[2],
                                                                                              np.dot(array_matrix[0],
                                                                                                     array_matrix[
                                                                                                         1])))))
        # print(T0_6)
        return T0_6

    def matrix_dot(self, array_matrix, num1, num2):
        #global matrix
        matrix = None
        if num1 == 0:
            if num2 == 1:
                matrix = array_matrix[0].dot(array_matrix[1])
            if num2 == 2:
                matrix = (array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])
            if num2 == 3:
                matrix = ((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])
            if num2 == 4:
                matrix = (((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])
            if num2 == 5:
                matrix = ((((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])
            if num2 == 6:
                matrix = (((((array_matrix[0].dot(array_matrix[1])).dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])).dot(array_matrix[6])
        elif num1 == 1:
            if num2 == 2:
                matrix = array_matrix[1].dot(array_matrix[2])
            if num2 == 3:
                matrix = (array_matrix[1].dot(array_matrix[2])).dot(array_matrix[3])
            if num2 == 4:
                matrix = ((array_matrix[1].dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])
            if num2 == 5:
                matrix = (((array_matrix[1].dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])
            if num2 == 6:
                matrix = ((((array_matrix[1].dot(array_matrix[2])).dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])).dot(array_matrix[6])
        elif num1 == 2:
            if num2 == 3:
                matrix = array_matrix[2].dot(array_matrix[3])
            if num2 == 4:
                matrix = (array_matrix[2].dot(array_matrix[3])).dot(array_matrix[4])
            if num2 == 5:
                matrix = ((array_matrix[2].dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])
            if num2 == 6:
                matrix = (((array_matrix[2].dot(array_matrix[3])).dot(array_matrix[4])).dot(array_matrix[5])).dot(array_matrix[6])
        elif num1 == 3:
            if num2 == 4:
                matrix = array_matrix[3].dot(array_matrix[4])
            if num2 == 5:
                matrix = (array_matrix[3].dot(array_matrix[4])).dot(array_matrix[5])
            if num2 == 6:
                matrix = ((array_matrix[3].dot(array_matrix[4])).dot(array_matrix[5])).dot(array_matrix[6])
        elif num1 == 4:
            if num2 == 5:
                matrix = array_matrix[4].dot(array_matrix[5])
            if num2 == 6:
                matrix = (array_matrix[4].dot(array_matrix[5])).dot(array_matrix[6])
        elif num1 == 5:
            if num2 == 6:
                matrix = array_matrix[5].dot(array_matrix[6])
        else:
            matrix = "Error: "
        return matrix

    def angular_Euler_calculation(self, transform_matrix):
        # global theta, fi, psi
        rotation_matrix = transform_matrix[0:3, 0:3]
        r3_3 = transform_matrix[2, 2]
        r2_3 = transform_matrix[1, 2]
        r1_3 = transform_matrix[0, 2]
        r3_2 = transform_matrix[2, 1]
        r3_1 = transform_matrix[2, 0]
        r1_1 = transform_matrix[0, 0]
        r2_1 = transform_matrix[1, 0]
        r1_2 = transform_matrix[0, 1]
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

        return [theta, fi, psi]

    def calculate_inverse_kinematics_problem(self, array_matrix):
        # print(array_matrix)
        T0_1 = self.matrix_dot(array_matrix, 0, 1)
        T0_4 = self.matrix_dot(array_matrix, 0, 4)

        p4 = np.array(T0_4[0:3, 3])  # вектор p4, который содержит координаты пересечения осей поворота двух последних
        # звеньев
        p1 = np.array(T0_1[0:3, 3]) # вектор p1, для нахождения начала координат первого сочленения

        a_length = self.length_vector(p1, p4) # Длина вектора a
        x0_4 = None # ToDo: Дописать
        y0_4 = None
        #c = sqrt((x0_4) ** 2+(y0_4) ** 2)
        #a = sqrt(x1_4 ** 2 + y1_4 ** 2 + z1_4 ** 2)
       # print(T0_4)
        #print("T_04 \n")
       # print(a_length)

    def length_vector(self, point_A, point_B):
        length = sqrt((point_A[0] - point_B[0]) ** 2 + (point_A[1] - point_B[1]) ** 2 + (point_A[2] - point_B[2]) ** 2)
        return length
    def take_coordinate(self, xyz, number_of_matrix1, number_of_matrix2):
        T = self.matrix_create()


