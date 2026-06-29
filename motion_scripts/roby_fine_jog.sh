#!/usr/bin/env bash
# Lance l'outil de jog fin / calibration nid sur le PC.
# Force le Python SYSTEME (3.12, ROS) et NON pyenv (3.11 sans tkinter/rclpy).
set -e

# Neutralise pyenv pour cette session
unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | paste -sd ':')"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
unset GTK_PATH 2>/dev/null || true   # evite le crash GUI sous env snap

exec /usr/bin/python3 "$HOME/roby_fine_jog.py"
