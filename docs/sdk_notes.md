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

## Проверка в WSL2: 2026-10-19

Подготовлена Ubuntu 22.04 x86_64, Python 3.10.12, GCC 11.4 и CMake 3.22.
Установлены build-essential, cmake, libeigen3-dev и ca-certificates.
WSL использует mirrored networking. Ethernet ноутбука: 192.168.123.100/24,
в Linux это eth1. По адресу 192.168.123.161 отвечает устройство с MAC
7E-1D-75-60-F5-89, но его роль и версия прошивки AS2 не подтверждены.
Проверка SSH из Linux получила connection refused.

Официальный unitree_sdk2 скачан в .runtime/unitree_sdk2, commit:
`d1abf001220d3e5f59e00a0c4aaa595e00089a4a`.
SDK установлен только в .runtime/unitree-sdk-install; примеры отключены.
Наш as2_state_probe успешно собран в build/as2-probe.
Первый реальный GetState через eth1 вернул 3104: SDK определяет это как
UT_ROBOT_ERR_CLIENT_API_TIMEOUT. Получение состояния пока НЕ подтверждено.
Канал DDS инициализировался локально, что само по себе не подтверждает связь с роботом.

Повторяемая сборка из корня checkout в Ubuntu:

```bash
bash deploy/build_state_probe.sh
```

Скрипт использует уже скачанный SDK, не устанавливает системные пакеты
и не обращается к роботу. Отдельный read-only запрос:

```bash
timeout 15 ./build/as2-probe/as2_state_probe eth1
```

Probe ждёт 2 секунды на DDS discovery, затем делает один GetState
с таймаутом 5 секунд. API состояния не привязан к IP .161: SDK использует
DDS domain 0 на выбранном интерфейсе. Безопаснее проводить проверку в
изолированной сети с одним роботом. Не запускать примеры движения для
диагностики таймаута. Следующие направления проверки: разрешённый DDS-трафик
в Windows/Hyper-V firewall, активный SDK-сервис и версия API/прошивки AS2.

Повторный GetState после паузы discovery и с таймаутом 5 секунд также
вернул 3104. У Hyper-V firewall WSL обнаружена политика входящих Block.
Проведён отдельный тест с временным inbound UDP-разрешением портов 7400-7500
только от 192.168.123.161; ответ по-прежнему не получен. Правило удалено
в finally. Это не исключает влияние других фильтров, портов или настройки
DDS, и не доказывает, что SDK-сервис на роботе выключен.

## Уточнение по официальному quick-start

Страница [Quick Development](https://support.unitree.com/home/en/AS2_SDK_Development_Guide/quick_development)
прочитана через браузер 2026-10-19. Она прямо называет 192.168.123.161
адресом основной платы AS2: это подтверждает назначение IP из инструкции,
но не версию прошивки подключённого устройства. Указаны Ubuntu 20.04/22.04,
отсутствие обязательной зависимости SDK2 от ROS2 и рекомендация физического
Linux при отсутствии опыта настройки WSL/виртуальных сетей.

В скачанном SDK есть отдельный официальный read-only пример
`example/as2/sport/as2_sport_state.cpp`: подписка на `rt/lf/sportmodestate`.
Для него используется отдельная сборка с BUILD_EXAMPLES=ON и только target
as2_sport_state. Системная установка SDK для этого не требуется.

Важно: `as2_sport_client` не является безопасным меню для проверки связи.
В commit d1abf001220d3e5f59e00a0c4aaa595e00089a4a начальный id равен 1;
после ввода `list` он остаётся прежним, и цикл вызывает BalanceStand.
Поэтому запуск этого примера и ввод list без готовности к изменению позы
не допускается. Числовые команды из скриншотов не заменяют чтение исходников.

Установлены оставшиеся зависимости quick-start: libyaml-cpp-dev,
libboost-all-dev, libfmt-dev. Официальный as2_sport_state успешно собран:

```bash
cmake -S .runtime/unitree_sdk2 -B build/unitree-official -DBUILD_EXAMPLES=ON
cmake --build build/unitree-official --target as2_sport_state --parallel 2
timeout 20 ./build/unitree-official/bin/as2_sport_state eth1
```

Результат проверки 2026-10-19: за 20 секунд подписчик не вывел телеметрию.
Перед запуском ping 192.168.123.161 прошёл без потерь, задержка 0-1 мс.
Запуск as2_sport_client и команды позы/движения не выполнялись. Это
подтверждает отсутствие полученных данных и в официальном примере, но
не локализует причину: остаются WSL/multicast/фильтрация, SDK-сервис и
совместимость прошивки. Инструкция рекомендует физическую установку Linux
при сложностях с сетью WSL; проверка на обычном Ubuntu-компьютере позволит
отделить эту причину от проблем на стороне робота.

## Успешная связь через WSL: 2026-10-19

Проверка пакетов локализовала проблему входящей фильтрации. В WSL были
видны исходящие DDS discovery-пакеты, но не ответы. Ограниченный Ethernet
захват Windows через netsh trace показал RTPS от 192.168.123.161, в том числе
unicast на ноутбук: порт назначения в этой сессии был 45967. Поэтому
предыдущий тест только диапазона 7400-7500 не охватывал весь DDS-трафик.

Временные разрешения inbound UDP только от 192.168.123.161 были добавлены
одновременно для физического Ethernet в Windows firewall и для WSL в
Hyper-V firewall. С ними GetState вернул 0, а официальный as2_sport_state
за 5 секунд вывел 368 строк телеметрии. Получены поля fsm_name=NORMAL_WALK,
process_state=0, speed_level=1, позиция и скорость. Название режима не
означает, что роботу отправлялась команда ходьбы: вызывались только GetState
и подписка. Оба временных правила удалены после проверки.

Этот тест подтвердил работоспособность совокупности разрешений, но не
изолировал, какое из двух правил необходимо само по себе. Постоянные
правила пока не включены. После их удаления связь может снова давать 3104.

Для проверенной конфигурации добавлен deploy/configure_dds_firewall.ps1.
В PowerShell от администратора из корня проекта:

```powershell
.\deploy\configure_dds_firewall.ps1 -Action Status
.\deploy\configure_dds_firewall.ps1 -Action Enable -WhatIf
.\deploy\configure_dds_firewall.ps1 -Action Enable
.\deploy\configure_dds_firewall.ps1 -Action Disable
```

Enable создаёт постоянные разрешения UDP только от IP робота. Для Windows
правило ограничено интерфейсом Ethernet, для Hyper-V — средой WSL. TCP,
SSH, другие адреса и глобальные firewall-политики не изменяются. Поскольку
разрешён весь UDP от этого адреса, применять только в доверенной сети
робота и удалять через Disable после работы. Если выполнение PowerShell
сценариев ограничено политикой организации, не менять системную политику:
согласовать выполнение с администратором. Захваты и журналы в logs не
публиковать: они могут содержать локальную диагностику и телеметрию.

После отдельного согласия владельца постоянные правила включены через
`configure_dds_firewall.ps1 -Action Enable`. Проверены Enabled=True для
AS2-VLM-Robot-UDP-Ethernet и AS2-VLM-Robot-UDP-WSL. Повторный GetState
через eth1 вернул 0. Эти правила остаются включёнными до явного Disable;
глобальные политики firewall не отключались. При работе в другой сети
правила следует отключить, поскольку ограничение доверяет IP .161,
а не криптографически подтверждённой идентичности робота.
