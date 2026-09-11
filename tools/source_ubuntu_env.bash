#!/usr/bin/env bash
# Source the Ubuntu ROS 2 + project Python + optional local ROS overlay.
# Usage: source tools/source_ubuntu_env.bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Please source this file: source tools/source_ubuntu_env.bash" >&2
    exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
local_ros_prefix="${repo_root}/.ros-overlay/opt/ros/jazzy"

source /opt/ros/jazzy/setup.bash
source "${repo_root}/.venv-ros/bin/activate"
# The system ROS underlay is already loaded above.  local_setup.bash adds only
# this workspace and avoids replaying stale chained underlays from setup.bash.
source "${repo_root}/install-venv/local_setup.bash"

if [[ -d "${local_ros_prefix}" ]]; then
    export AMENT_PREFIX_PATH="${local_ros_prefix}:${AMENT_PREFIX_PATH:-}"
    export COLCON_PREFIX_PATH="${local_ros_prefix}:${COLCON_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${local_ros_prefix}/lib:${LD_LIBRARY_PATH:-}"
    local_python_packages="${local_ros_prefix}/lib/python3.12/site-packages"
    if [[ -d "${local_python_packages}" ]]; then
        export PYTHONPATH="${local_python_packages}:${PYTHONPATH:-}"
    fi
fi

export REBOTARM_WORKSPACE="${repo_root}"
echo "reBotArm Ubuntu environment ready: ${repo_root}"
