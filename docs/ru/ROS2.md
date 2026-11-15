# Топики и launch-файлы ROS 2 (Jazzy)

[English version](../en/ROS2.md)

terrasim работает без ROS. ROS 2 нужен, только чтобы видеть идущую симуляцию в виде топиков или запускать её через
`ros2 launch`.

## 1. `--ros`: топики идущего прогона

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/drive.sh --engine harmonic --robot husky --traj square:L=4,rc=0.6 --ros --gui
# в другом терминале
ros2 topic list
ros2 topic echo /terrasim/wheel_current
GZ_PARTITION=terrasim gz topic -l
```

`--ros` (включает `--jobs 1`):
* пишет лог построчно (`flush_log = 1`);
* запускает `ros2/terrasim_bridge.py`, который следит за `<прогон>/log.csv` и публикует **для любого движка** (Gazebo
  Harmonic или эмулятор):

| топик | тип | что |
|---|---|---|
| `/terrasim/ground_truth` | `nav_msgs/Odometry` | поза (система карты) и скорость корпуса модели |
| `/terrasim/odom` | `nav_msgs/Odometry` | одометрия по энкодерам, посчитанная в модели робота |
| `/terrasim/imu` | `sensor_msgs/Imu` | IMU модели (со своей моделью шума) |
| `/terrasim/wheels` | `sensor_msgs/JointState` | по колёсам: скорость энкодера (velocity), момент мотора на колесе (effort) |
| `/terrasim/wheel_current` | `std_msgs/Float64MultiArray` | ток мотора по колёсам / бортам, А |
| `/terrasim/wheel_slip` | `std_msgs/Float64MultiArray` | продольное проскальзывание по колёсам |
| `/terrasim/wheel_sinkage` | `std_msgs/Float64MultiArray` | осадка по колёсам, м |
| `/terrasim/wheel_force` | `std_msgs/Float64MultiArray` | опора, тяга, сила свободного качения по колёсам, Н |
| `/terrasim/body_contact` | `std_msgs/Float64MultiArray` | наклон корпуса к нормали земли (°), опора корпуса о землю (Н), сколько точек корпуса касаются земли |
| `/clock` | `rosgraph_msgs/Clock` | только эмулятор: время симуляции из лога |

* в Gazebo Harmonic дополнительно фиксирует партицию Gazebo `terrasim` (или `--gz-partition ИМЯ`), добавляет в мир
  штатные IMU, OdometryPublisher и JointStatePublisher (`--gz-sensors`) и запускает `ros_gz_bridge` для `/clock`,
  `/imu`, `/model/<робот>/odometry`, `/joint_states`.

Токи моторов, проскальзывание, осадка, силы контакта и контакт корпуса — величины terrasim: у Gazebo для них датчиков нет, поэтому они
доступны только через узел-мост. Штатный `/model/<робот>/odometry` Gazebo — это истинная поза модели; одометрия колёс
с проскальзыванием — `/terrasim/odom`.

Мост можно запустить и вручную на любой папке прогона (например, чтобы «проиграть» законченный прогон):

```bash
python3 ros2/terrasim_bridge.py --ros-args -p log:=runs_drive/<папка>/<прогон>/log.csv -p clock:=true
```

## 2. Почему без `--ros` в `ros2 topic list` пусто
* каждый прогон Gazebo идёт в своей партиции `GZ_PARTITION=terrasim_<pid>_<слот>`, чтобы параллельные прогоны
  (`--jobs N`) не видели друг друга; `gz topic -l` в другом терминале смотрит в партицию по умолчанию и ничего не видит;
* пакетные прогоны мост не запускают: они идут без окна как можно быстрее и только читают лог.

## 3. Launch-файлы

```bash
ros2 launch ~/terrasim/ros2/drive.launch.py robot:=husky traj:=circle:R=2 ros:=true gui:=true
ros2 launch ~/terrasim/ros2/drive.launch.py robot:=husky map:=slope_15@across gui:=true ros:=true
ros2 launch ~/terrasim/ros2/drive.launch.py robot:=youbot traj:=figure8:A=1.2 heading:=fixed control:=open,closed
ros2 launch ~/terrasim/ros2/robotino.launch.py surfaces:=carpet speeds:=0.2 gui:=true rtf:=5
ros2 launch ~/terrasim/ros2/robotino.launch.py engine:=emulate surfaces:=all noise:=clean
```

Аргументы `drive.launch.py`: `engine` (harmonic | emulate, по умолчанию harmonic), `robot`, `map`, `relief`, `floor`,
`surface`, `map_size`, `res`, `spawn`, `traj`, `speed`, `accel`, `drive`, `heading`, `control`, `localization`,
`motor_control`, `imu_noise`, `gui`, `rtf`, `jobs`, `out`, `ros` (все топики terrasim + штатные топики gz),
`bridge` (только штатные топики gz), `extra` (любые другие ключи `tools/drive_run.py` через пробел).

Аргументы `robotino.launch.py`: `engine`, `surfaces`, `speeds`, `dirs`, `t_run`, `gui`, `rtf`, `jobs`, `noise`, `map`,
`out`, `quick`, `extra`.

Launch-файлы вызывают те же скрипты, что и командная строка, поэтому любой ключ из [RUNNING](RUNNING.md) доступен через
`extra`. Если проект лежит не в `~/terrasim`, укажите путь к своей копии.
