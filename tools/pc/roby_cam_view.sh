#!/usr/bin/env bash
# Visualiseur LIVE des 2 cameras dans le navigateur -> http://localhost:8081/
# Relais MJPEG des topics ROS (aucun acces materiel, aucun conflit avec le Pi5).
set -e

# rclpy = python systeme 3.12 (pas le venv lerobot, pas pyenv)
unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | grep -v '/venv/' | paste -sd ':')"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"

exec /usr/bin/python3 "$HOME/roby_cam_view.py"
