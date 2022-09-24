from manipulator import Manipulator

############## Настройки программы ##############
baud = 115200
teensy_port = 5
arduino_port = 3
################# Конец настроек #################

robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)

robot.jog_joint(robot.joints[0], 10, 20)
print(robot.calculate_inverse_kinematic_problem([[0.1],
                                                [0.1],
                                                [0.1]]))


