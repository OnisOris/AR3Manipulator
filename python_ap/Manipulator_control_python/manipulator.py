import serial
import time
import math
from loguru import logger
from joint import Joint
from config import DEFAULT_SETTINGS


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
            # robot_code = str(self.serial_teensy.readline())
            # pcode = robot_code[2:4]  # TODO: разобраться с pcode
            # if pcode == "01":
            #     apply_robot_calibration(robot_code)
        else:
            logger.warning(f"Joint {joint.number_joint} AXIS LIMIT")

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

        for i in range(6):
            DEFAULT_SETTINGS[f'J{i + 1}Open_loop_val'] = None

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
        for joint in self.joints:
            if joint.get_current_joint_angle() == 0:
                joint.current_joint_angle = 0.0001

        wrist_config = "F" if self.joints[4].current_joint_angle > 0 else "N"
        C4 = math.radians(float(self.joints[1].current_joint_angle) + DEFAULT_SETTINGS['DH_t_1'])
        C5 = math.radians(float(self.joints[2].current_joint_angle) + DEFAULT_SETTINGS['DH_t_2'])
        C6 = math.radians(float(self.joints[3].current_joint_angle) + DEFAULT_SETTINGS['DH_t_3'])
        C7 = math.radians(float(self.joints[4].current_joint_angle) + DEFAULT_SETTINGS['DH_t_4'])
        C8 = math.radians(float(self.joints[5].current_joint_angle) + DEFAULT_SETTINGS['DH_t_5'])
        C9 = math.radians(float(self.joints[6].current_joint_angle) + DEFAULT_SETTINGS['DH_t_6'])
