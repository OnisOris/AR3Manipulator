import serial

global teensy_serial
global arduino_serial


def move_to(command):
    teensy_serial.write(command.encode())


def set_teensy_port(teensy_port, baud) -> None:
    global serial_teensy
    port = f"COM{str(teensy_port)}"
    serial_teensy = serial.Serial(port, baud)


def set_arduino_port(arduino_port, baud) -> None:
    global serial_arduino
    port = f"COM{str(arduino_port)}"
    serial_arduino = serial.Serial(port, baud)


def J1jogPos(speed, ACCdur, ACCspd, DECdur, DECspd, J1Degs):


def main():
    ############## Настройки программы ##############
    baud = 115200
    teensy_port = 11
    arduino_port = 12
    ################# Конец настроек #################
    c1 = "MJA1444S25G15H10I20K5U0V2321W3X7599Y2281Z5313" + "\n"
    c2 = "MJA13065B13762C13915D0577E0660F0148T10S25G15H10I20K5U10663V6084W3915X7023Y1627Z3164\n"
    c3 = "MJA10B01109C11D00E00F01T10S25G15H10I20K5U10663V4975W3916X7023Y1627Z3163\n"
    c4 = "MJA01775B11110C00D10E0218F00T10S25G15H10I20K5U8888V6085W3916X7023Y1409Z3163\n"
    # Вызов функций, необходимых для инициализации работы
    set_arduino_port(arduino_port, baud)
    set_teensy_port(teensy_port, baud)


if __name__ == "__main__":
    main()

# def calibration():
