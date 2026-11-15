# ROS 2 (Jazzy) topics and launch files

[Русская версия](../ru/ROS2.md)

terrasim runs without ROS. ROS 2 is only needed to see a running simulation as topics or to start it with
`ros2 launch`.

## 1. `--ros`: topics of a running job

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/drive.sh --engine harmonic --robot husky --traj square:L=4,rc=0.6 --ros --gui
# in another shell
ros2 topic list
ros2 topic echo /terrasim/wheel_current
GZ_PARTITION=terrasim gz topic -l
```

`--ros` (implies `--jobs 1`):
* writes the log row by row (`flush_log = 1`);
* starts `ros2/terrasim_bridge.py`, which follows `<run>/log.csv` and publishes, for **either engine** (Gazebo
  Harmonic or the emulator):

| topic | type | content |
|---|---|---|
| `/terrasim/ground_truth` | `nav_msgs/Odometry` | pose (map frame) and body velocity of the simulated robot |
| `/terrasim/odom` | `nav_msgs/Odometry` | encoder odometry computed in the robot model |
| `/terrasim/imu` | `sensor_msgs/Imu` | IMU of the model (with its noise model) |
| `/terrasim/wheels` | `sensor_msgs/JointState` | per wheel: encoder speed (velocity), motor torque at the wheel (effort) |
| `/terrasim/wheel_current` | `std_msgs/Float64MultiArray` | motor current per wheel / side, A |
| `/terrasim/wheel_slip` | `std_msgs/Float64MultiArray` | longitudinal slip per wheel |
| `/terrasim/wheel_sinkage` | `std_msgs/Float64MultiArray` | sinkage per wheel, m |
| `/terrasim/wheel_force` | `std_msgs/Float64MultiArray` | normal force, traction, free-rolling force per wheel, N |
| `/terrasim/body_contact` | `std_msgs/Float64MultiArray` | tilt of the chassis to the ground normal (°), body-ground normal force (N), body points touching the ground |
| `/clock` | `rosgraph_msgs/Clock` | emulator only: simulation time of the log |

* on Gazebo Harmonic it also fixes the Gazebo partition to `terrasim` (or `--gz-partition NAME`), adds the native gz
  IMU sensor, OdometryPublisher and JointStatePublisher to the world (`--gz-sensors`) and starts `ros_gz_bridge` for
  `/clock`, `/imu`, `/model/<robot>/odometry`, `/joint_states`.

Motor currents, slip, sinkage, contact forces and the body contact are terrasim quantities: Gazebo has no sensors for them, so they are
available only through the bridge node. The native `/model/<robot>/odometry` of Gazebo is the true pose of the model;
the wheel odometry with slip is `/terrasim/odom`.

The bridge node can also be started by hand on any run folder (for example to replay a finished run):

```bash
python3 ros2/terrasim_bridge.py --ros-args -p log:=runs_drive/<folder>/<run>/log.csv -p clock:=true
```

## 2. Why `ros2 topic list` is empty without `--ros`
* every Gazebo job runs in its own partition `GZ_PARTITION=terrasim_<pid>_<slot>`, so that parallel jobs (`--jobs N`)
  do not see each other; a `gz topic -l` in another shell uses the default partition and sees nothing;
* batch runs start no bridge: they run headless as fast as possible and only read the log back.

## 3. Launch files

```bash
ros2 launch ~/terrasim/ros2/drive.launch.py robot:=husky traj:=circle:R=2 ros:=true gui:=true
ros2 launch ~/terrasim/ros2/drive.launch.py robot:=husky map:=slope_15@across gui:=true ros:=true
ros2 launch ~/terrasim/ros2/drive.launch.py robot:=youbot traj:=figure8:A=1.2 heading:=fixed control:=open,closed
ros2 launch ~/terrasim/ros2/robotino.launch.py surfaces:=carpet speeds:=0.2 gui:=true rtf:=5
ros2 launch ~/terrasim/ros2/robotino.launch.py engine:=emulate surfaces:=all noise:=clean
```

`drive.launch.py` arguments: `engine` (harmonic | emulate, default harmonic), `robot`, `map`, `relief`, `floor`,
`surface`, `map_size`, `res`, `spawn`, `traj`, `speed`, `accel`, `drive`, `heading`, `control`, `localization`,
`motor_control`, `imu_noise`, `gui`, `rtf`, `jobs`, `out`, `ros` (all terrasim topics + native gz topics),
`bridge` (native gz topics only), `extra` (any other option of `tools/drive_run.py`, space separated).

`robotino.launch.py` arguments: `engine`, `surfaces`, `speeds`, `dirs`, `t_run`, `gui`, `rtf`, `jobs`, `noise`, `map`,
`out`, `quick`, `extra`.

The launch files call the same scripts as the command line, so every option of [RUNNING](RUNNING.md) is available
through `extra`. If the project is not in `~/terrasim`, use the path of your clone.
