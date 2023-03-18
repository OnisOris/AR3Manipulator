# AR3 manipulator

В проекте используется манипулятор, созданный
Chris Annin.

https://github.com/Chris-Annin

https://www.anninrobotics.com/

Прошивка для teensy и arduino mega используется с официального сайта. 

Приложение для управления находится в папке:

ManipulatorControl/python_ap/Manipulator_control_python/

## Кинематическая схема

В проекте используется своя кинематическая схема, которая отличается от оригинальной. 
<div align="center">
  <img src="/img/kscheme.jpg" alt="">
</div>


## Команды

Перемещение всех звеньев в заданные углы:

``` move_all -90 0 0 0 0 0```

Вывод в консоль текущих углов и положений в глобальной системе координат

```print```

## Примеры

