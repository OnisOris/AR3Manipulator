import numpy
import serial
import time
import math
from math import (sin, cos, pi)
from loguru import logger
from joint import Joint
from config import DEFAULT_SETTINGS
import numpy as np


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

        for i, joint in self.joints:
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

        DEFAULT_SETTINGS['calculated_direction'] = None
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

    def calculate_direct_kinematics_problem(self):
        # for joint in self.joints:
        #     if joint.get_current_joint_angle() == 0:
        #         joint.current_joint_angle = 0.0001
        Transform_matrix = self.matrix_create()

    def matrix_create(self):
        cja = [float(self.joints[0].current_joint_angle), float(self.joints[1].current_joint_angle),
               float(self.joints[2].current_joint_angle),
               float(self.joints[3].current_joint_angle), float(self.joints[4].current_joint_angle),
               float(self.joints[5].current_joint_angle)]
        TS = {'a_1': 64.20, 'a_2': 0, 'a_3': 0, 'a_4': 0, 'a_5': 0, 'a_6': 0,
              'alpha_1': math.pi / 2, 'alpha_2': 0, 'alpha_3': math.pi / 2, 'alpha_4': -math.pi / 2,
              'alpha_5': math.pi / 2, 'alpha_6': 0,
              'd_1': 169.77, 'd_2': 0, 'd_3': 0, 'd_4': 222.63, 'd_5': 0, 'd_6': 36.25,
              'displacement_theta_3': math.pi / 2}
        T = []
        for i in range(0, 6):
            T.append(np.array(
                [[cos(cja[i]), -sin(cja[i]) * cos(TS[f'alpha_{i + 1}']), sin(cja[i]) * sin(TS[f'alpha_{i + 1}']),
                  TS[f'a_{i + 1}'] * cos(cja[i])],
                 [sin(cja[i]), cos(cja[i]) * cos(TS[f'alpha_{i + 1}']), -cos(cja[i]) * sin(TS[f'alpha_{i + 1}']),
                  TS[f'a_{i + 1}'] * sin(cja[i])],
                 [0, sin(cja[i]), cos(cja[i]), TS[f'd_{i + 1}']],
                 [0, 0, 0, 1]]))
            #print(T[i])
        return self.matrix_dot_all(T)
    def matrix_dot_all(self, array_matrix):
        T0_6 = np.dot(array_matrix[5], np.dot(array_matrix[4], np.dot(array_matrix[3], np.dot(array_matrix[2], np.dot(array_matrix[0], array_matrix[1])))))
        print(T0_6)
        return T0_6

