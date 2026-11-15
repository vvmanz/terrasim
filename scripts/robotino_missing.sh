#!/usr/bin/env bash
# Run the (surface, speed) series that a Robotino dataset folder does not have yet and merge them in.
#   bash scripts/robotino_missing.sh datasets/my_robotino_set                  # Gazebo Harmonic, defaults: all surfaces x 0.1,0.2,0.3
#   bash scripts/robotino_missing.sh datasets/my_robotino_set --jobs 4
#   bash scripts/robotino_missing.sh datasets/my_robotino_set --dry-run        # only list what is missing
# -> <folder>_missing (new series), <folder>_full (all together). Details: python3 tools/robotino_missing.py --help
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec python3 tools/robotino_missing.py "$@"
