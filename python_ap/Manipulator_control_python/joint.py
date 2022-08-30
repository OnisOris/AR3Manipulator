from loguru import logger


class Joint:

    def __init__(self, number_joint, positive_angle_limit, negative_angle_limit, step_limit):
        self.motor_direction = 0
        self.number_joint = number_joint
        self.positive_angle_limit = float(positive_angle_limit)
        self.negative_angle_limit = float(negative_angle_limit)
        self.step_limit = float(step_limit)
        # TODO: Исправить значение current_joint_step на значение из файла калибровок (и остальных ниже)
        self.current_joint_step = 0
        self.current_joint_angle = 0
        self.degrees_per_step = (self.positive_angle_limit - self.negative_angle_limit) / self.step_limit

    def get_current_joint_step(self):
        return self.current_joint_step

    def get_current_joint_angle(self):
        return self.current_joint_angle
