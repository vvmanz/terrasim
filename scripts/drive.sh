#!/usr/bin/env bash
# Multi-robot sandbox: the built-in robots (or your own, configs/robots) drive trajectories over terrain maps.
# All arguments go to tools/drive_run.py (python3 tools/drive_run.py --help for the full list, docs/en/RUNNING.md).
#
#   bash scripts/drive.sh --list                                                          # robots, surfaces, maps, trajectories
#   bash scripts/drive.sh --engine harmonic --robot robotino --floor checkerboard:surfaces=gray+carpet,cell=0.5 --traj circle:R=1
#   bash scripts/drive.sh --engine harmonic --robot husky --relief craters:n=5,R=1.5,depth=0.15,flat0=1.5 --traj square:L=4,rc=0.6 --gui
#   bash scripts/drive.sh --engine harmonic --robot youbot --traj figure8:A=1.2 --heading fixed --control open,closed
#   bash scripts/drive.sh --engine emulate  --robot robotino,youbot,husky,rover --localization truth,odom,odom_imu --jobs 4
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
ENGINE=emulate
ARGS=("$@")
for x in "${ARGS[@]}"; do case "$x" in --list|--help|-h) exec python3 tools/drive_run.py "$@";; esac; done
for ((k = 0; k < ${#ARGS[@]}; k++)); do
  [ "${ARGS[$k]}" = "--engine" ] && ENGINE="${ARGS[$((k + 1))]}"
  case "${ARGS[$k]}" in --engine=*) ENGINE="${ARGS[$k]#--engine=}";; esac
done
mkdir -p logs
# --ros: the topics need the ROS 2 environment for any engine (rclpy, ros_gz_bridge)
for x in "${ARGS[@]}"; do [ "$x" = "--ros" ] && [ -f /opt/ros/jazzy/setup.bash ] && { set +u; source /opt/ros/jazzy/setup.bash; }; done
[ -x core/build/vehicle3d ] || bash scripts/build.sh core > logs/build_core_drive.txt 2>&1 || { echo "core build failed: logs/build_core_drive.txt"; exit 1; }
case "$ENGINE" in
  harmonic)
    if [ -f /opt/ros/jazzy/setup.bash ]; then set +u; source /opt/ros/jazzy/setup.bash; fi
    [ -f build/harmonic/libterramech_gz_system.so ] || bash scripts/build.sh harmonic
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build/harmonic:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    command -v gz > /dev/null || { echo "gz not found (Gazebo Harmonic not installed?)"; exit 1; }
    ;;
esac
exec python3 tools/drive_run.py "$@"
