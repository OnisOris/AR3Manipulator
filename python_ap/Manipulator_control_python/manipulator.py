import serial
import time
from loguru import logger
from joint import Joint


class Manipulator:

    def __init__(self, teensy_port, arduino_port, baud):
        self.ACC_dur = 15
        self.ACC_spd = 10
        self.DEC_dur = 20
        self.DEC_spd = 5
        self.joint_jog_degrees = 10
        self.joints = [Joint(i+1) for i in range(6)]

        try:

            self.serial_teensy: serial.Serial = serial.Serial(f"COM{teensy_port}", baud)
            self.serial_arduino: serial.Serial = serial.Serial(f"COM{arduino_port}", baud)

        except serial.SerialException:
            logger.error("Serial port not defined")
            self.serial_teensy = None
            self.serial_arduino = None

    def move_to(self, command):
        self.serial_teensy.write(command.encode())

    def Jjog(self, joint: Joint, Jdrivedir, j_jog_steps, speed, ACCdur, ACCspd, DECdur, DECspd, J1Degs, JogStepsStat, degrees):
        # global JogStepsStat
        global J1StepCur
        global J2StepCur
        global J3StepCur
        global J4StepCur
        global J5StepCur
        global J6StepCur
        global J1AngCur
        #joint_jog_degrees
        if JogStepsStat == 0:
            j_jog_steps = int(degrees / joint.degrees_per_step)  # высчитываем количенство шагов, joint.degrees_per_step -
        else:
            # switch from degs to steps
            j_jog_steps = degrees
            degrees *= joint.degrees_per_step
        drive_direction = 1 if joint.motor_direction == 0 else 0
        if degrees <= (joint.positive_angle_limit-joint.current_joint_angle):
            joint.current_joint_step += int(j_jog_steps)
            joint.current_joint_angle = round(joint.negative_angle_limit + (joint.current_joint_step * joint.degrees_per_step))
            # save_position_data() TODO:ДОдлЕАТЬ save_position_data() и calculate_direct_kinematics_problem()
            # calculate_direct_kinematics_problem()
            command = f"MJ{Joint}{Jdrivedir}{j_jog_steps}S{speed}G{ACCdur}H{ACCspd}I{DECdur}K{DECspd}U{J1StepCur}V{J2StepCur}" \
                  f"W{J3StepCur}X{J4StepCur}Y{J5StepCur}Z{J6StepCur}\n"
            self.teensy_push(command)
            logger.debug(f"Write to teensy: {command}")
            self.serial_teensy.flushInput()
            time.sleep(.2)
            robot_code = str(self.serial_teensy.readline())
            # pcode = robot_code[2:4]  # TODO: разобраться с pcode
            # if pcode == "01":
            #     apply_robot_calibration(robot_code)
        else:
            logger.warning(f"Joint {joint.number_joint} AXIS LIMIT")


    def teensy_push(self, command):
        self.serial_teensy.write(command.encode())

    def arduino_push(self):
        self.serial_arduino.write(self.command.encode())
