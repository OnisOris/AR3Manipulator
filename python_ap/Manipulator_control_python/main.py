import time
from manipulator import Manipulator


robot = Manipulator(test_mode=False)


robot.auto_calibrate()

robot.null_position()

time.sleep(5)

robot.read_points('./Scripts_UV/opening_cap.txt')

robot.read_points('./Scripts_UV/calibration.txt')

robot.read_points('./Scripts_UV/take_cuvette_1.txt')

robot.read_points('./Scripts_UV/move_to_UV.txt')

robot.read_points('./Scripts_UV/take_cuvette_2.txt')

robot.read_points('./Scripts_UV/closing_cap.txt')

robot.read_points('./Scripts_UV/calibration.txt')

robot.read_points('./Scripts_UV/opening_cap.txt')

robot.read_points('./Scripts_UV/calibration.txt')

robot.read_points('./Scripts_UV/moving_to_base.txt')

