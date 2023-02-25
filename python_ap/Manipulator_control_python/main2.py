import math
import time
from config import DEFAULT_SETTINGS
from manipulator import Manipulator, Position
import numpy as np
from math import (pi)
from loguru import logger
from pynput import keyboard
#import keyboard

############## Настройки программы ##############
baud = 1152001
teensy_port = 3
arduino_port = 4
################# Конец настроек #################
robot = Manipulator(f'COM{teensy_port}', f'COM{arduino_port}', baud)
print ("Начало")
def on_press(key):
    try:
        print(f'Нажата буквенно-цифровая клавиша: {key.char}')
    except AttributeError:
        print(f'Нажата специальная клавиша: {key}')

def on_release(key):
    #grad = 10
    print(f'{key} released')
    key2 = f'{key}'
    if key == keyboard.Key.ctrl_l:
        robot.delta -=3
        print(f'Значение перемещения {robot.delta}')
    if key == keyboard.Key.shift:
        robot.delta +=3
        print(f'Значение перемещения {robot.delta}')
    if key == keyboard.Key.left:
        robot.jog_joint(robot.joints[0], 20, -robot.delta)
    if key == keyboard.Key.right:
        robot.jog_joint(robot.joints[0], 20, robot.delta)
    if key == keyboard.Key.page_up:
        robot.jog_joint(robot.joints[1], 20, -robot.delta)
    if key == keyboard.Key.page_down:
        robot.jog_joint(robot.joints[1], 20, robot.delta)
    if key == keyboard.Key.up:
        robot.jog_joint(robot.joints[2], 20, -robot.delta)
    if key == keyboard.Key.down:
        robot.jog_joint(robot.joints[2], 20, robot.delta)
    if key2 == '<100>':
        robot.jog_joint(robot.joints[3], 20, robot.delta)
    if key2 == '<102>':
        robot.jog_joint(robot.joints[3], 20, -robot.delta)
    if key2 == '<98>':
        robot.jog_joint(robot.joints[4], 20, robot.delta)
    if key2 == '<104>':
        robot.jog_joint(robot.joints[4], 20, -robot.delta)
    if key2 == '<103>':
        robot.jog_joint(robot.joints[5], 20, robot.delta)
    if key2 == '<97>':
        robot.jog_joint(robot.joints[5], 20, -robot.delta)
    if key == keyboard.Key.f1:
        robot.auto_calibrate()
    if key == keyboard.Key.space:
        robot.grab()
    if key == keyboard.Key.alt_l:
        robot.absolve()
    if key == keyboard.Key.f2:
        robot.print()
    if key == keyboard.Key.esc:
        robot.finish()
        # Возврат False - остановит слушатель
        return False

# блок `with` слушает события до выхода
# до остановки слушателя
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

#...или неблокирующим способом:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()