# AS2: проверенные источники и следующий запуск

Проверено 2026-10-09. Это сведения об опубликованном SDK, а не о подключённом роботе.

## Доступность документации

Пользователь предоставил https://support.unitree.com/home/en/AS2_SDK_Development_Guide/service_interface . В браузере загрузились название Software Interface Services и дата 2026-08-04 17:50:50, но article пуст и разделы руководства не появились. Поэтому сетевые адреса, SSH и сенсорные topics из этой страницы не подтверждены.

## Подтверждено исходниками Unitree

- [README SDK](https://github.com/unitreerobotics/unitree_sdk2/blob/main/README.md): prebuild Ubuntu 20.04 LTS, aarch64/x86_64, GCC 9.4.0; поддержана установка в собственный prefix.
- [AS2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/as2/sport/sport_client.hpp): отдельный namespace `unitree::robot::as2`, методы `Move(float,float,float)`, `StopMove()`, `GetState(std::map<std::string,std::string>&)`.
- [CMake SDK](https://github.com/unitreerobotics/unitree_sdk2/blob/main/CMakeLists.txt): C++17, архитектурные библиотеки, экспорт пакета `unitree_sdk2`.

Исходники прочитаны с ветки main; конкретный commit пока не закреплён. Перед сборкой на целевой машине записать commit предоставленного SDK и сверить API. `GetState` не считается источником метрической одометрии: структура возвращаемой map ещё не проверена. Номера gait и параметры управления не определялись.

## Preflight без сети и движения

На целевом Linux-компьютере из корня проекта:

```bash
python3 -m as2_vlm.preflight --sdk-root /path/to/unitree_sdk2
```

Путь заменить на фактический checkout официального SDK. Скрипт не устанавливает пакеты, не меняет сеть и не подключается к роботу. Если Python ниже 3.10, приложение пока не совместимо: на Ubuntu 20.04 штатный Python может быть старше требуемого. Отдельный Python runtime подготавливается после проверки среды, системный Python не заменять.

## Сборка read-only probe

После установки совместимого SDK в отдельный prefix, например `/opt/unitree_robotics`:

```bash
cmake -S robot/bridge_cpp -B build/as2-probe -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
cmake --build build/as2-probe
./build/as2-probe/as2_state_probe --help
```

Реальный запрос состояния запускается отдельно с проверенным именем интерфейса:

```bash
./build/as2-probe/as2_state_probe <network_interface>
```

Утилита вызывает только Init и GetState; она не вызывает Move, StandUp, SwitchJoystick или смену режима. Не меняет владельца управления. Здесь её сборка и подключение ещё не проверены: нет Linux SDK/CMake и робота.

Следующий шаг после preflight: подтвердить SDK connection, получить реальные источники RGB/геометрии/одометрии и добавить sensor adapter. Адреса и права доступа нужны от владельца; публичное руководство не раскрывает настройки конкретной машины.
