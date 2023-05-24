from loguru import logger


class Joint:

    def __init__(self, number_joint, endstop_angle, angle_limit, step_limit):
        self.name_joint = None
        self.motor_direction = 0
        self.motor_dir = 1  # позже нужно удалить
        self.number_joint = number_joint
        self.angle_limit = float(angle_limit)
        self.endstop_angle = float(endstop_angle)
        self.step_limit = int(step_limit)
        self.open_loop_stat = False
        self.current_joint_step = 0
        self.current_joint_angle = 0
        self.degrees_per_step = (abs(self.endstop_angle) + abs(self.angle_limit)) / self.step_limit  # град/шаг

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

    def __str__(self):
        return f'Joint_{self.number_joint}({self.get_name_joint()})'


