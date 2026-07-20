#!/usr/bin/env bash
# Lance le panneau de collecte dataset (tri 1-par-1) sur le PC.
# Python SYSTEME (3.12, ROS/tkinter/rclpy), PAS pyenv. Env ROS domaine 42 + DDS.
set -e

unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | paste -sd ':')"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
unset GTK_PATH 2>/dev/null || true

exec /usr/bin/python3 "$HOME/roby_collect_panel.py"
