from manipulator import Manipulator
robot = Manipulator(test_mode=False)
robot.read_points('/Scripts/calibration.txt')
robot.read_points('/Scripts/opening_cap.txt')
