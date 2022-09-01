from loguru import logger


class Joint:

    # def __init__(self, number_joint, positive_angle_limit, negative_angle_limit, step_limit):
    def __init__(self, number_joint, positive_angle_limit, negative_angle_limit, step_limit):
        self.name_joint = None
        self.positive_angle_limit = None
        self.negative_angle_limit = None
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

    def change_current_joint_step(self, current_joint_step) -> None:
        self.current_joint_step = current_joint_step

    def change_current_joint_angle(self, current_joint_angle) -> None:
        self.current_joint_angle = current_joint_angle

    def set_name_joint(self, name_joint):
        self.name_joint = name_joint

    def get_name_joint(self) -> str:
        return self.name_joint


