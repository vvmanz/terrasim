#!/usr/bin/env bash
# Build core (+tests) and the Gazebo plugin.
#   scripts/build.sh harmonic   # Ubuntu 24.04 + Gazebo Harmonic (gz-sim 8, DART)
#   scripts/build.sh core       # only core + offline tools (no Gazebo needed)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FLAVOR="${1:-core}"
cd "$ROOT"
mkdir -p core/build build logs
echo "== core =="
CXX="${CXX:-g++}"
$CXX -O2 -std=c++17 -Wall -shared -fPIC -I core/include core/src/c_api.cpp -o core/build/libterramech_c.so
$CXX -O2 -std=c++17 -Wall -I core/include core/src/robotino2d.cpp -o core/build/robotino2d
$CXX -O2 -std=c++17 -Wall -I core/include core/tests/test_core.cpp -o core/build/test_core
$CXX -O2 -std=c++17 -Wall -I core/include core/tests/robotino3d.cpp -o core/build/robotino3d
$CXX -O2 -std=c++17 -Wall -I core/include core/tests/vehicle3d.cpp -o core/build/vehicle3d
./core/build/test_core | tee logs/test_core.txt

case "$FLAVOR" in
  harmonic)
    # Gazebo installed via ROS 2 Jazzy (vendor packages) needs the ROS environment for CMake to find gz-sim8
    if [ -f /opt/ros/jazzy/setup.bash ]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
    cmake -S gz_harmonic -B build/harmonic -DCMAKE_BUILD_TYPE=Release 2>&1 | tee logs/cmake_harmonic.txt
    cmake --build build/harmonic -j"$(nproc)" 2>&1 | tee logs/build_harmonic.txt
    ls -la build/harmonic/libterramech_gz_system.so
    ;;
  core) ;;
  *) echo "unknown flavor $FLAVOR"; exit 1;;
esac
echo "BUILD OK ($FLAVOR)"
