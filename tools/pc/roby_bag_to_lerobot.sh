#!/usr/bin/env bash
# Wrapper : source ROS (pour rosbag2_py) PUIS lance le python du venv XPU (pour lerobot).
# Les deux mondes cohabitent dans le meme interpreteur (python 3.12 des deux cotes).
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
exec "$HOME/ipex_test_venv/bin/python" "$HOME/ros2_ws/tools/pc/roby_bag_to_lerobot.py" "$@"
