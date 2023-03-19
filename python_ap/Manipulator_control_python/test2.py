from manipulator import Manipulator, Position
from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi
from loguru import logger
np.set_printoptions(precision=3, suppress=True)
# d a alpha theta
DH = {
        'a_1': 0.0642,
        'a_2': 0.305,
        'a_3': 0.0,
        'a_4': 0.0,
        'a_5': 0.0,
        'a_6': 0.0,
        'alpha_1': pi*0.5,
        'alpha_2': 0.0,
        'alpha_3': pi*0.5,
        'alpha_4': -pi*0.5,
        'alpha_5': pi*0.5,
        'alpha_6': 0.0,
        'd_1': 0.16977,
        'd_2': 0.0,
        'd_3': 0.0,
        'd_4': 0.22263,
        'd_5': 0.0,
        'd_6': 0.125,
        'displacement_theta_1': 0.0,
        'displacement_theta_2': 0.0,
        'displacement_theta_3': pi/2,
        'displacement_theta_4': 0.0,
        'displacement_theta_5': 0.0,
        'displacement_theta_6': 0.0
    }
# dh_params = np.array([[0.16977, 0.0642, 0.5 * pi, 0.],
#                       [0., 0.305, 0.0, 0.],
#                       [0., 0.6005, pi*0.5, 0],
#                       [0.22263, 0., -0.5 * pi, 0.],
#                       [0., 0., 0.5 * pi, 0.],
#                       [0.125, 0., 0., 0.]])
# logger.debug(dh_params)

# dh_params = np.array([[DH['d_1'], DH['a_1'], DH['alpha_1'], DH['displacement_theta_1']],
#                       [DH['d_2'], DH['a_2'], DH['alpha_2'], DH['displacement_theta_2']],
#                       [DH['d_3'], DH['a_3'], DH['alpha_3'], DH['displacement_theta_3']],
#                       [DH['d_4'], DH['a_4'], DH['alpha_4'], DH['displacement_theta_4']],
#                       [DH['d_5'], DH['a_5'], DH['alpha_5'], DH['displacement_theta_5']],
#                       [DH['d_6'], DH['a_6'], DH['alpha_6'], DH['displacement_theta_6']]
#                       ])
# logger.debug(dh_params)
# robot = RobotSerial(dh_params)

# # =====================================
# # forward
# # =====================================
#
# theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# f = robot.forward(theta)
#
# # print("-------forward-------")
# # print("end frame t_4_4:")
# # print(f.t_4_4)
# print("end frame xyz:")
# print(f.t_3_1.reshape([3, ]))
# print("end frame abc:")
# print(f.euler_3)
# print("end frame rotational matrix:")
# print(f.r_3_3)
# print("end frame quaternion:")
# print(f.q_4)
# print("end frame angle-axis:")
# print(f.r_3)

# robot.show()

# =====================================
# inverse
# =====================================

# xyz = np.array([[0.3], [0.5], [0.1]])
# abc = np.array([-0., 0., 0.]) # x x x
# end = Frame.from_euler_3(abc, xyz)
# robot.inverse(end)
#
# print("inverse is successful: {0}".format(robot.is_reachable_inverse))
# print("axis values: \n{0}".format(robot.axis_values))
# robot.show()
# logger.debug(robot.axis_values)
# # example of unsuccessful inverse kinematics
# xyz = np.array([[2.2], [0.], [1.9]])
# end = Frame.from_euler_3(abc, xyz)
# robot.inverse(end)
#
# print("inverse is successful: {0}".format(robot.is_reachable_inverse))