#!/usr/bin/env bash
# Affiche la zone de travail (datasets) dans RViz, sur /roby/reperes.
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# --ros-args est indispensable : sans lui rclpy ignore les -p et garde les defauts.
exec /usr/bin/python3 "$HOME/ros2_ws/tools/pc/roby_marqueur_zone.py" --ros-args "$@"
