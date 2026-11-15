"""ROS 2 launch wrapper for the multi-robot sandbox (same as scripts/drive.sh).

    ros2 launch ~/terrasim/ros2/drive.launch.py robot:=robotino traj:=circle:R=1 floor:=checkerboard:surfaces=gray+carpet,cell=0.5
    ros2 launch ~/terrasim/ros2/drive.launch.py robot:=husky relief:="craters:n=5,R=1.5,depth=0.15,flat0=1.5" traj:=square:L=4,rc=0.6 gui:=true
    ros2 launch ~/terrasim/ros2/drive.launch.py robot:=youbot traj:=figure8:A=1.2 heading:=fixed control:=open,closed
    ros2 launch ~/terrasim/ros2/drive.launch.py robot:=rover relief:="rough:A=0.04,L=3,flat0=1" gui:=true bridge:=true
    ros2 launch ~/terrasim/ros2/drive.launch.py robot:=burger traj:=circle:R=0.8 floor:=uniform:surface=carpet_shag gui:=true
    ros2 launch ~/terrasim/ros2/drive.launch.py robot:=husky map:=slope_15@across gui:=true ros:=true
        (bridge:=true: native gz IMU + OdometryPublisher in the world and ros_gz_bridge to ROS topics
         /imu (sensor_msgs/Imu) and /model/<robot>/odometry (nav_msgs/Odometry); one run at a time)

Arguments (all optional): engine (harmonic|emulate), robot, map (a map file, NAME or NAME@scenario), relief, floor, surface, map_size, res,
spawn (x,y,yaw_deg), traj, speed, accel, drive, heading, control, localization, motor_control, imu_noise,
gui, rtf, jobs, out, bridge (true|false: native gz IMU/odometry only), ros (true|false: all terrasim topics, docs/en/ROS2.md),
extra (any other options of tools/drive_run.py, space separated).
Batch: comma lists in robot/control/localization/heading/motor_control, '|' between traj/floor/relief variants.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARGS = [('engine', 'harmonic'), ('robot', 'robotino'), ('map', ''), ('relief', ''), ('floor', ''), ('surface', ''), ('map_size', ''), ('res', ''),
        ('spawn', ''), ('traj', ''), ('speed', ''), ('accel', ''), ('drive', ''), ('heading', ''), ('control', ''), ('localization', ''),
        ('motor_control', ''), ('imu_noise', ''), ('gui', 'false'), ('rtf', ''), ('jobs', '1'), ('out', ''), ('bridge', 'false'), ('ros', 'false'), ('extra', '')]
FLAGS = {k: '--' + k.replace('_', '-') for k, _ in ARGS if k not in ('engine', 'robot', 'gui', 'jobs', 'bridge', 'ros', 'extra')}
PARTITION = 'terrasim_drive'


def _true(v):
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


def build_command(get):
    cmd = ['bash', os.path.join(ROOT, 'scripts', 'drive.sh'), '--engine', get('engine'), '--robot', get('robot'), '--jobs', get('jobs')]
    for k, flag in FLAGS.items():
        if get(k): cmd += [flag, get(k)]
    if _true(get('gui')): cmd.append('--gui')
    if _true(get('ros')): cmd += ['--ros']                                   # terrasim topics + native gz topics (docs/en/ROS2.md)
    elif _true(get('bridge')): cmd += ['--gz-sensors', '--gz-partition', PARTITION, '--jobs', '1']
    if get('extra'): cmd += get('extra').split()
    return cmd


def bridge_command(get):
    robots = [r for r in get('robot').split(',') if r]
    topics = ['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU', '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    topics += [f'/model/{r}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' for r in robots]
    return ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + topics


def _setup(context, *args, **kwargs):
    get = lambda k: LaunchConfiguration(k).perform(context)
    acts = [ExecuteProcess(cmd=build_command(get), output='screen', cwd=ROOT)]
    if _true(get('bridge')) and not _true(get('ros')) and get('engine') == 'harmonic':
        acts.append(ExecuteProcess(cmd=bridge_command(get), output='screen', additional_env={'GZ_PARTITION': PARTITION}))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS] + [OpaqueFunction(function=_setup)])
