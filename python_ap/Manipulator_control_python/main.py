from manipulator import Manipulator
robot = Manipulator(test_mode=False)
robot.read_points('./Scripts_UV/calibration.txt')
robot.read_points('./Scripts_UV/opening_cap.txt')
