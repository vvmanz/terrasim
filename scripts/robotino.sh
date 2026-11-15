#!/usr/bin/env bash
# Robotino "star" experiments on any surfaces -> datasets in the format of the Robotino lab recordings (docs/en/DATASETS.md).
# All arguments go to tools/robotino_run.py (python3 tools/robotino_run.py --help for the full list).
#
#   bash scripts/robotino.sh --list                                            # surface library
#   bash scripts/robotino.sh --engine harmonic --surfaces real                 # 4 calibrated floors (Gazebo Harmonic)
#   bash scripts/robotino.sh --engine harmonic --surfaces synthetic --jobs 4   # synthetic floors, 4 worlds in parallel
#   bash scripts/robotino.sh --engine harmonic --surfaces hard --noise clean    # hard floors: tile, linoleum, concrete, wet, ice
#   bash scripts/robotino.sh --engine harmonic --surfaces carpet --speeds 0.2 --gui --rtf 5   # watch, 5x real time
#   bash scripts/robotino.sh --engine emulate  --surfaces all                  # no Gazebo, same physics code
#   bash scripts/robotino.sh --engine harmonic --surfaces gray --quick         # 12 directions x 6 s, pipeline check
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
ENGINE=harmonic
ARGS=("$@")
for x in "${ARGS[@]}"; do case "$x" in --list|--help|-h) exec python3 tools/robotino_run.py "$@";; esac; done
for ((k = 0; k < ${#ARGS[@]}; k++)); do
  [ "${ARGS[$k]}" = "--engine" ] && ENGINE="${ARGS[$((k + 1))]}"
  case "${ARGS[$k]}" in --engine=*) ENGINE="${ARGS[$k]#--engine=}";; esac
done
mkdir -p logs
[ -x core/build/robotino3d ] || bash scripts/build.sh core > logs/build_core_robotino.txt 2>&1 || { echo "core build failed: logs/build_core_robotino.txt"; exit 1; }
case "$ENGINE" in
  harmonic)
    if [ -f /opt/ros/jazzy/setup.bash ]; then set +u; source /opt/ros/jazzy/setup.bash; fi
    [ -f build/harmonic/libterramech_gz_system.so ] || bash scripts/build.sh harmonic
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build/harmonic:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    command -v gz > /dev/null || { echo "gz not found (Gazebo Harmonic not installed?)"; exit 1; }
    ;;
esac
mkdir -p logs
exec python3 tools/robotino_run.py "$@"
