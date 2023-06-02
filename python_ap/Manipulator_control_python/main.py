from manipulator import Manipulator

############## Настройки программы ##############
baud = 115200
teensy_port = 3
arduino_port = 5
################# Конец настроек #################


robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud, camera=True, controller_dualshock=False,
                    continuouse_mesurement=False, checking_chanhing_of_angles=False)
robot.start_program()
