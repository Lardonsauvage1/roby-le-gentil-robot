#!/usr/bin/env bash
# Panneau du bras guide : bouton de recentrage + activation du joystick.
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec /usr/bin/python3 "$(dirname "$(readlink -f "$0")")/roby_leader_panel.py"
