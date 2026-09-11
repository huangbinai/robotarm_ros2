#!/usr/bin/env bash
# Run colcon with the project interpreter so console-script shebangs match it.
set -eo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${task_root}/.venv-ros/bin/activate"
cd "${task_root}"
"${task_root}/.venv-ros/bin/python" -c \
  'from colcon_core.command import main; raise SystemExit(main())' \
  build --base-paths src --build-base build-venv --install-base install-venv \
  --symlink-install "$@" --cmake-args "-DPython3_EXECUTABLE=${task_root}/.venv-ros/bin/python"
"${task_root}/.venv-ros/bin/python" tools/check_python_entrypoints.py
