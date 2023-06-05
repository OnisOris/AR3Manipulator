# AR3 manipulator

В проекте используется манипулятор, созданный
Chris Annin.

https://github.com/Chris-Annin

https://www.anninrobotics.com/

Прошивка для teensy и arduino mega используется с официального сайта. 

Приложение для управления находится в папке:

ManipulatorControl/python_ap/Manipulator_control_python/

## Старт манипулятора

Проверьте подключение кабелей от arduino, teensy и камеры к блоку управления и к ноутбуку.

<div align="center">
  <img src="/img/cabels.png" alt="">
</div>

Включите питание манипулятора.

<div align="center">
  <img src="/img/on_off_0.png" alt="">
</div>

Если нажата кнопка экстренного отключения двигателей, отожмите ее, провернув по часовой стрелке.

<div align="center">
  <img src="/img/on_off_1.png" alt="">
</div>

Проверьте порты от teensy и arduino. По необходимости внесите правки в main файле: смените arduino_port и teensy_port. Arduino в диспетчере устройств обычно отображается с CH340.

<div align="center">
  <img src="/img/device_manager.png" alt="">
</div>

## Запуск консоли

Данный пример находится в файле console.py 

Импортируем класс Manipulator

```from manipulator import Manipulator```

Задаем скорость связи с микроконтроллерами

```baud = 115200```

Записываем порты микроконтроллеров (выяснить их номера можно по диспетчеру устройств)

```teensy_port = 3```

```arduino_port = 5```

Создаем экземпляр класса

```robot = Manipulator()```

Используем метод, запускающий консоль, в которую мы можем записывать команды (смотрите ниже пункт "Команды в консоли")

```robot.start_program()```

## Пример использования различных файлов со скриптами
Файл с данным примером находится в main.py

``` import time ```


```from manipulator import Manipulator```


```robot = Manipulator(test_mode=False)```


```robot.auto_calibrate()```

```robot.null_position()```

```time.sleep(5)```

```robot.read_points('./Scripts_UV/opening_cap.txt')```

```robot.read_points('./Scripts_UV/calibration.txt')```

```robot.read_points('./Scripts_UV/take_cuvette_1.txt')```

```robot.read_points('./Scripts_UV/move_to_UV.txt')```

```robot.read_points('./Scripts_UV/take_cuvette_2.txt')```

```robot.read_points('./Scripts_UV/closing_cap.txt')```

```robot.read_points('./Scripts_UV/calibration.txt')```

```robot.read_points('./Scripts_UV/opening_cap.txt')```

```robot.read_points('./Scripts_UV/calibration.txt')```

```robot.read_points('./Scripts_UV/moving_to_base.txt')```

## Команды в консоли

Перемещение всех звеньев в заданные углы:

``` move_all -90 0 0 0 0 0```

Вывод в консоль текущих углов и положений в глобальной системе координат

```print```

Перемещение вдоль оси x, y, z:
```move_x [значение, в мм]```

```move_y [значение, в мм]```

```move_z [значение, в мм]```

Управление схватом, значение от 0 до 180

```servo [значение]```

Сжать схват

```grab```

Отпустить схват

```absolve```

Перемещение по координатам. x, y, z - декартовы координаты. 

```txmove [x] [y] [z] [a_z] [a_y] [a_x]```

Перемещение по углам каждого из джойнтов

```move_all [a] [b] [c] [d] [e] [f]```

Добавление последней точки ОЗК

```add```

Чтение команд с файла points

```read```

Команда калибровки отдельных джойнтов, выбирается шесть чисел, порядок чисел соответствует порядку джойнтов, 1 -
калибровка, 0 - пропуск калибровки. Пример представлен ниже

```calib_axe 101010```

## Команды, выполняемые с файла points.txt


```inv,[x],[y],[z],[a],[b],[c]```


```move_z,[z]```
```move_y,[y]```
```move_x,[x]```

```move_xyz,[x],[y],[z]```

```rest```

camr - функция калибровки по маркерам. 

id маркера - номер арукомаркера.

Высота над маркером - высота, которая указывается в глобальной системе координат (относительно манипулятора, а не маркера).

state - поворот гриппера по оси z на 180 град (гриппер ориентируется вдоль оси x, либо в противоположную сторону).

delta_x - смещение отностительно системы координат камеры (ось x направлена влево, ось y вниз). 

```camr,[id маркера],[высота над маркером],[число циклов калибровки],[state (1 или 0)],[delta_x, delta_y]```

Поворот по углам каждого из шести джойнтов.

```dir,[a],[b],[c],[d],[e],[f]```

Сжатие схвата.

```grab```

Разжатие схвата.

```abs```

Калибровка всех звеньев.

```calib```

Функция остановки выполнения кода, время указывается в секундах.

```sleep,[время [с]]```

```speed,[скорость, от 1 до 35]```

Пример программы представлен в файле points.txt

## Кинематическая схема

В проекте используется своя кинематическая схема, которая отличается от оригинальной. 
<div align="center">
  <img src="/img/scheme.png" alt="">
</div>

