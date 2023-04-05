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
  <img src="/img/scheme.png" alt="">
</div>


## Команды

Перемещение всех звеньев в заданные углы:

``` move_all -90 0 0 0 0 0```

Вывод в консоль текущих углов и положений в глобальной системе координат

```print```

```move_x```

```move_y```

```move_z```

```servo```

```grab```

```absolve```

```move```

```move_all```

```add```

```save```

```txmove```

```vis```

```vis_on```

```vis_off```

```read```

```calib_axe```

## Примеры

