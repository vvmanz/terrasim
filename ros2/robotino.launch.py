"""ROS 2 launch wrapper for the Robotino surface experiments (same as scripts/robotino.sh).

    ros2 launch ~/terrasim/ros2/robotino.launch.py                                  # 4 real floors, Harmonic
    ros2 launch ~/terrasim/ros2/robotino.launch.py surfaces:=synthetic jobs:=4
    ros2 launch ~/terrasim/ros2/robotino.launch.py surfaces:=carpet speeds:=0.2 gui:=true rtf:=5
    ros2 launch ~/terrasim/ros2/robotino.launch.py surfaces:=gray map:=hills:A=0.012,L=0.4 gui:=true
    ros2 launch ~/terrasim/ros2/robotino.launch.py engine:=emulate surfaces:=all noise:=clean
    ros2 launch ~/terrasim/ros2/robotino.launch.py surfaces:=carpet extra:="--sweep carpet:mu=0.6,0.8,1.0"

Arguments (all optional): engine (harmonic|emulate|surrogate), surfaces, speeds, dirs, t_run,
gui (true|false), rtf, jobs, noise (clean|noisy|both), map, out, quick (true|false), extra (any other
options of tools/robotino_run.py, space separated). Output: datasets/<date>_<engine>/ (see README).
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARGS = [('engine', 'harmonic'), ('surfaces', 'real'), ('speeds', ''), ('dirs', ''), ('t_run', ''), ('gui', 'false'),
        ('rtf', ''), ('jobs', '1'), ('noise', 'both'), ('map', ''), ('out', ''), ('quick', 'false'), ('extra', '')]
FLAGS = {'speeds': '--speeds', 'dirs': '--dirs', 't_run': '--t-run', 'rtf': '--rtf', 'map': '--map', 'out': '--out'}


def _true(v):
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


def build_command(get):
    cmd = ['bash', os.path.join(ROOT, 'scripts', 'robotino.sh'), '--engine', get('engine'), '--surfaces', get('surfaces'),
           '--jobs', get('jobs'), '--noise', get('noise')]
    for k, flag in FLAGS.items():
        if get(k): cmd += [flag, get(k)]
    if _true(get('gui')): cmd.append('--gui')
    if _true(get('quick')): cmd.append('--quick')
    if get('extra'): cmd += get('extra').split()
    return cmd


def _setup(context, *args, **kwargs):
    get = lambda k: LaunchConfiguration(k).perform(context)
    return [ExecuteProcess(cmd=build_command(get), output='screen', cwd=ROOT)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS] + [OpaqueFunction(function=_setup)])
