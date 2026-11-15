#!/usr/bin/env bash
# Install build/runtime dependencies.
#   scripts/install_deps.sh            # Ubuntu 24.04: gz-sim 8 dev packages (OSRF apt repo) unless ROS 2 Jazzy vendor pkgs exist
set -euo pipefail
FLAVOR="${1:-harmonic}"
sudo apt-get update
sudo apt-get install -y build-essential cmake python3-numpy python3-scipy python3-matplotlib python3-pandas python3-yaml python3-tabulate
if [ -d /opt/ros/jazzy/opt/gz_sim_vendor ]; then
  echo "Gazebo from ROS 2 Jazzy vendor packages detected; ensuring dev parts are present"
  sudo apt-get install -y ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor ros-jazzy-gz-cmake-vendor ros-jazzy-gz-common-vendor || true
elif ! dpkg -s libgz-sim8-dev >/dev/null 2>&1; then
  sudo apt-get install -y curl lsb-release gnupg
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y gz-harmonic libgz-sim8-dev libgz-plugin2-dev libgz-cmake3-dev libgz-common5-dev
fi
echo "deps OK ($FLAVOR)"
