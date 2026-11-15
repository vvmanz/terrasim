# Примеры: скопировать, запустить, поменять

[English version](../en/EXAMPLES.md)

Каждая команда на этой странице запускалась ровно в таком виде. Возьмите ту, что ближе к вашей задаче, и меняйте её.
Все команды запускаются из папки проекта (`cd ~/terrasim`). Без `--engine` они идут в эмуляторе (секунды, Gazebo не
нужен); добавьте `--engine harmonic --gui` к любой команде `drive.sh`, чтобы посмотреть тот же прогон в Gazebo.

Содержание:
1. [Первые запуски и где результаты](#1-первые-запуски-и-где-результаты)
2. [Робот + карта + траектория + параметры](#2-робот--карта--траектория--параметры)
3. [Свой робот по шагам](#3-свой-робот-по-шагам)
4. [Своя поверхность по шагам](#4-своя-поверхность-по-шагам)
5. [Своя карта по шагам](#5-своя-карта-по-шагам)
6. [Всё своё в одном прогоне](#6-всё-своё-в-одном-прогоне)
7. [Падения и опрокидывания](#7-падения-и-опрокидывания)
8. [Датасеты Robotino](#8-датасеты-robotino)
9. [Как прочитать результаты в Python](#9-как-прочитать-результаты-в-python)

Готовые файлы, которые используются ниже: [`examples/robots/`](../../examples/robots) (три робота),
[`examples/maps/my_first_map.yaml`](../../examples/maps/my_first_map.yaml),
[`examples/my_surfaces.json`](../../examples/my_surfaces.json), [`examples/replay_twist.csv`](../../examples/replay_twist.csv).

## 1. Первые запуски и где результаты

```bash
bash scripts/drive.sh --list                                    # что есть: роботы, поверхности, траектории, карты
bash scripts/drive.sh --robot husky --traj circle:R=2           # Husky, круг радиусом 2 м, серый резиновый пол
bash scripts/drive.sh --robot husky --traj circle:R=2 --surface sand_lete    # то же на песке
bash scripts/drive.sh --robot jackal --map hill@contour         # Jackal на готовой карте (карта hill, сценарий contour)
```

Команда печатает по строке на прогон и папку с результатами `runs_drive/<дата>_<движок>/`:

```
runs_drive/20261116_101500_emulate/
  husky_circle_diff_closed_truth/figure.png     <- открыть первым: путь, ошибки, токи, проскальзывание, IMU
  husky_circle_diff_closed_truth/log.csv        <- всё по времени (50 Гц)
  husky_circle_diff_closed_truth/metrics.json   <- числа прогона
  summary.csv, summary.md                        <- по строке на прогон (удобно для пакетов)
```

`--out моя_папка` кладёт результаты в другое место. Столбцы описаны в [OUTPUTS](OUTPUTS.md).

## 2. Робот + карта + траектория + параметры

Команда собирается из независимых частей. Меняйте по одной:

```
bash scripts/drive.sh  --robot husky  --map slope_15@up  --speed 0.8  --set robot.mass=70  --engine harmonic --gui
                       └─ кто ──────┘ └─ где ─────────┘  └─ как ────────────────────────┘ └─ движок ──────────────┘
```

| часть | ключи | где читать |
|---|---|---|
| кто | `--robot` (встроенный, файл робота, список через запятую) | [ROBOTS](ROBOTS.md) |
| где | `--map ИМЯ@сценарий` или `--relief` + `--floor` + `--surface`, `--spawn` | [MAPS](MAPS.md) |
| что проехать | `--traj` (или сценарий карты), `--speed`, `--accel`, `--heading` | [RUNNING §3](RUNNING.md#3-траектории) |
| как | `--control`, `--localization`, `--motor-control`, `--imu-noise`, `--set ...` | [RUNNING](RUNNING.md) |
| физика | `--body-contact`, `--on-tip`, `--surface-file`, `--set <поверхность>.<параметр>` | [PHYSICS](PHYSICS.md), [SURFACES](SURFACES.md) |
| движок | `--engine emulate` (по умолчанию) или `harmonic`, `--gui`, `--rtf`, `--jobs` | [RUNNING §9](RUNNING.md#9-движки-скорость-и-воспроизводимость) |

### 2.1 Карты и сценарии
```bash
bash scripts/drive.sh --robot husky --map slope_15@up                        # вверх по склону 15°
bash scripts/drive.sh --robot husky,jackal --map "slope_15@*" --jobs 4       # 2 робота × 4 сценария = 8 прогонов, по 4 сразу
bash scripts/drive.sh --robot rover --map mars_yard                          # ровер на «марсианском полигоне» (первый сценарий)
bash scripts/drive.sh --robot husky --map slope_10@up --surface-file configs/surfaces_husky.json --surface gravel_fit
#                                     та же карта, но покрытие по умолчанию заменено подобранным гравием из библиотеки шин
```
`python3 tools/maps.py list` показывает все карты со сценариями; картинки — в [`docs/maps/`](../maps).

### 2.2 Сгенерированный рельеф вместо карты
```bash
bash scripts/drive.sh --robot burger --relief "bumps:n=40,h=0.01,w=0.15,flat0=0.5" \
     --floor "checkerboard:surfaces=gray+carpet,cell=0.5" --traj figure8:A=1 --map-size 3
#    40 случайных кочек по 1 см, шахматка серый / ковролин, восьмёрка, карта ±3 м
```

### 2.3 Траектория, точка старта, скорость
```bash
bash scripts/drive.sh --robot husky --spawn=-2,1,45 --traj "spline:pts=1,0;2,1;3,0" --speed 0.8 --accel 1.0
#    старт в x=-2, y=1, курс 45°; сплайн через 3 точки стартовой системы; 0.8 м/с, 1 м/с²
bash scripts/drive.sh --robot youbot --traj "line:L=2,angle=90" --heading fixed
#    голономный робот едет 2 м боком (влево), не поворачиваясь
```
Отрицательные значения старта пишите через `=` (`--spawn=-2,1,45`), иначе оболочка примет их за ключ.

### 2.4 Управление, локализация, датчики (пакеты)
```bash
bash scripts/drive.sh --robot youbot --traj figure8:A=1.2 --heading fixed,tangent --control open,closed --jobs 4
#    2 курса × 2 режима = 4 прогона: сравнить без регулятора и с регулятором пути
bash scripts/drive.sh --robot robotino --traj square:L=2,rc=0 --localization truth,odom,odom_imu --imu-noise high --jobs 3
#    один квадрат с тремя обратными связями: истина, одометрия колёс, одометрия + гироскоп (шумный IMU)
bash scripts/drive.sh --robot husky --traj line:L=4 --motor-control voltage --surface clay_wet
#    моторы без контура скорости (U = Ce·ω_зад): в глине робот замедляется
```
Значения через запятую (или `|` для `--traj`, `--floor`, `--relief`) — это пакет: по прогону на каждое сочетание.
`summary.csv` их сравнивает.

### 2.5 Параметры робота на лету
```bash
bash scripts/drive.sh --robot husky --set robot.mass=80 --set robot.com_h=0.35 --map slope_20@across
#    груз 30 кг высоко, поперёк склона 20°
bash scripts/drive.sh --robot jackal --set motor.gear=30 --set wheel.patch=0.05 --traj square:L=3,rc=0.5
bash scripts/drive.sh --robot burger --set carpet.mu=0.6 --surface carpet --traj circle:R=0.5
#    --set меняет и параметры поверхности: здесь трение ковролина только на этот прогон
```

### 2.6 Воспроизведение записанных команд
```bash
bash scripts/drive.sh --robot youbot --replay examples/replay_twist.csv --replay-mode twist
#    в файле столбцы t, vx, vy, wz (скорость корпуса по времени); --replay-mode wheels берёт t, g0, g1, ... (скорости колёс)
```

## 3. Свой робот по шагам

Понадобятся: положения и радиусы колёс, масса и примерное положение центра масс, данные моторов (или разумные
оценки) и внешние габариты корпуса.

**Шаг 1. Взять пример.** В `examples/robots/` лежат три готовых файла робота:

| файл | что это |
|---|---|
| `my_diffbot.json` | два ведущих колеса + шаровая опора (как TurtleBot / Pioneer) |
| `my_skidsteer.json` | четыре колеса, бортовой поворот (как Husky / Jackal) |
| `husky_payload.json` | встроенный робот с несколькими изменёнными значениями (`"base": "husky"`) |

Скопируйте ближайший к вашему роботу в `configs/robots/` под именем своего робота:
```bash
cp examples/robots/my_diffbot.json configs/robots/my_bot.json
```

**Шаг 2. Поменять числа.** Файл (сокращённо):
```json
{
 "mass": 6.0,                          "com": [-0.03, 0.0, 0.08],
 "I": [0.05, 0.07, 0.09],              "Jw": 0.004,
 "size": [0.34, 0.30, 0.12],           "body_off": [-0.04, 0.0, 0.09],
 "drive": ["diff"],
 "wheels": [
  {"label": "L", "x": 0.0,  "y": 0.16,  "r": 0.06, "b": 0.03, "type": "standard", "group": 0},
  {"label": "R", "x": 0.0,  "y": -0.16, "r": 0.06, "b": 0.03, "type": "standard", "group": 1},
  {"label": "C", "x": -0.20, "y": 0.0,  "r": 0.02, "type": "caster", "group": -1, "swivel": 0}
 ],
 "motor": {"R": 1.5, "Ce": 0.02, "kt": 0.02, "gear": 50.0, "eta": 0.75, "I0": 0.15, "Vmax": 12.0, "kp": 0.1, "ki": 1.0}
}
```
* Система робота: начало на земле под центром робота, x вперёд, y влево, z вверх, метры.
* `com` — [x, y, высота] центра масс; `size` + `body_off` — ящик корпуса (им робот касается земли при падениях;
  низ ящика — на высоте реального клиренса).
* У каждого колеса: положение `x`, `y`, радиус `r`, ширина протектора `b`, `type` (`standard`, `omni`, `mecanum`,
  `caster`) и `group` — номер мотора (колёса с одним номером делят мотор; у опор −1).
* `motor` — из паспорта, формулы в [ROBOTS §3.3](ROBOTS.md#33-что-надо-знать). Неизвестные значения оставьте из
  примера и проверьте токи на рисунке.
* Все ключи — [ROBOTS §3](ROBOTS.md#3-свой-робот-полный-файл-робота).

**Шаг 3. Посмотреть на него.**
```bash
python3 tools/robot_preview.py --robots my_bot --body --out /tmp/prev      # /tmp/prev/my_bot.png
```
Красные окружности — радиусы колёс, с которыми считает физика; синие кубики — точки, которыми корпус касается земли.
Проверьте, что колёса стоят на земле, а корпус там, где нужно.

**Шаг 4. Первый прогон на ровном полу, потом сложнее.**
```bash
bash scripts/drive.sh --list | grep my_bot                    # он в списке вместе со встроенными
bash scripts/drive.sh --robot my_bot --traj circle:R=1        # ровный пол
bash scripts/drive.sh --robot my_bot --map slope_10@up        # склон
bash scripts/drive.sh --robot my_bot --map "examples/maps/my_first_map.yaml@*"   # ваш рельеф
bash scripts/drive.sh --robot my_bot --engine harmonic --gui --traj circle:R=1   # в Gazebo
```
Файл робота можно использовать и прямо по пути, не копируя: `--robot examples/robots/my_diffbot.json`.

**Шаг 5 (вместо 1–2). Поменять встроенного робота, а не писать нового.**
```bash
cp examples/robots/husky_payload.json configs/robots/husky_heavy.json
bash scripts/drive.sh --robot husky_heavy --map slope_20@up
```
`husky_payload.json` — это только `"base": "husky"` и то, что меняется (масса, центр масс, редуктор, пятно, скорость).

Частые ошибки:
* `z` колеса не равен радиусу у робота, чьё начало координат на земле (не пишите `z`: по умолчанию он равен `r`);
* оба ведущих колеса в одной `group` у дифференциального робота (тогда он может ехать только прямо);
* ящик `size` опускается ниже клиренса: корпус скребёт по земле на каждой кочке (смотрите синие кубики).

## 4. Своя поверхность по шагам

**Шаг 1.** Откройте [`examples/my_surfaces.json`](../../examples/my_surfaces.json). Поверхность — это `"base"`
(ближайшая существующая) плюс параметры, которые отличаются:
```json
"linoleum_soft": {"base": "table", "mu": 0.8, "f_lat": 0.012}
```
**Шаг 2.** Добавьте свою (смысл каждого параметра и как выбирать значения — [SURFACES](SURFACES.md#1-параметры)).

**Шаг 3.** Используйте:
```bash
bash scripts/drive.sh --robot burger --surface-file examples/my_surfaces.json --surface rubber_tiles --traj circle:R=0.8
bash scripts/drive.sh --robot husky --surface-file examples/my_surfaces.json --floor "stripes:surfaces=gray+gravel_like,w=1" --traj line:L=5
#    полосы серой резины и вашей «гравийной» поверхности шириной 1 м
```
В карте укажите имя в зоне и передайте тот же `--surface-file` или объявите поверхность прямо в карте (§5).

## 5. Своя карта по шагам

**Шаг 1.** Запустите пример как есть:
```bash
bash scripts/drive.sh --map examples/maps/my_first_map.yaml                      # сценарий road, роботы jackal + husky
bash scripts/drive.sh --map "examples/maps/my_first_map.yaml@*" --robot jackal --jobs 3   # все три сценария
python3 tools/maps.py preview examples/maps/my_first_map.yaml --out /tmp          # её картинка
```
**Шаг 2.** Скопируйте и правьте ([файл](../../examples/maps/my_first_map.yaml) прокомментирован построчно):
```bash
cp examples/maps/my_first_map.yaml configs/maps/my_map.yaml      # теперь работает --map my_map
# или начните с полного шаблона со всеми видами зон и рельефа:
python3 tools/maps.py new my_map
```
Части карты: `zones` (где какое покрытие), `relief` (высоты: холмы, склоны, кочки, кратеры, канавы...), `surfaces`
(поверхности только этой карты), `robots`, `scenarios` (старт + траектория). Все виды — [MAPS §2](MAPS.md#2-формат-файла).

**Шаг 3.** Проверьте и запустите:
```bash
python3 tools/maps.py check my_map --robot jackal,husky       # достаточно ли плавный рельеф для этих колёс?
bash scripts/drive.sh --map my_map
bash scripts/drive.sh --map my_map@hill --map-set relief.0.height=0.8 --robot husky    # попробовать значение без правки файла
```

## 6. Всё своё в одном прогоне

```bash
bash scripts/drive.sh --robot examples/robots/my_diffbot.json \
     --map examples/maps/my_first_map.yaml@bumps \
     --surface-file examples/my_surfaces.json --surface rubber_tiles
#    ваш робот, ваша карта (сценарий bumps), ваша поверхность как покрытие карты по умолчанию
```

## 7. Падения и опрокидывания

Корпус касается земли (`--body-contact on`, по умолчанию): робот садится брюхом на кочку, падает с уступа и
переворачивается. По умолчанию прогон останавливается через 0.2 с после того, как корпус наклонился к земле больше
чем на 50° (`--on-tip stop`); с `--on-tip continue` робот кувыркается дальше до конца прогона.

```bash
bash scripts/drive.sh --robot jackal --map rollover --on-tip continue     # боковой склон 55°: скатывается и лежит на крыше
bash scripts/drive.sh --robot jackal --map rollover@up --on-tip continue  # прямо вверх: опрокидывается назад
bash scripts/drive.sh --robot waffle --map washboard@across               # клиренс 1 см на волнах 2 см: скребёт брюхом
bash scripts/drive.sh --robot waffle --map washboard@across --body-contact off   # то же, но корпус проходит сквозь землю
bash scripts/drive.sh --robot husky --map rollover --on-tip continue --engine harmonic --gui
```
В логе есть наклон к земле и сила контакта корпуса (`tilt_rel`, `body_N`, `body_pts`), см.
[OUTPUTS](OUTPUTS.md#2-logcsv). Как это устроено — [PHYSICS §2.7](PHYSICS.md#27-контакт-корпуса-с-землёй-падения-и-опрокидывания).

## 8. Датасеты Robotino

```bash
bash scripts/robotino.sh --engine emulate --surfaces carpet --speeds 0.2 --quick         # 12 направлений, проверка цепочки
bash scripts/robotino.sh --engine emulate --surfaces real                                # 4 откалиброванных покрытия, полная звезда
bash scripts/robotino.sh --engine emulate --surface-file examples/my_surfaces.json --surfaces linoleum_soft --speeds 0.1 --quick
```
Формат и столбцы — [DATASETS](DATASETS.md).

## 9. Как прочитать результаты в Python

```python
import pandas as pd, json
run = 'runs_drive/<папка>/husky_circle_diff_closed_truth'
log = pd.read_csv(f'{run}/log.csv')                 # t, x, y, yaw, ..., w1_cur, w1_slip, ..., tilt_rel, body_N
print(log[['t', 'x', 'y', 'w1_cur', 'w1_slip']].describe())
m = json.load(open(f'{run}/metrics.json'))           # xtrack_rms_m, energy_J_per_m, slip_mean, odom_drift_m, ...
S = pd.read_csv('runs_drive/<папка>/summary.csv')    # по строке на прогон пакета
print(S[['run', 'status', 'progress', 'energy_J_per_m', 'slip_mean']])
```
