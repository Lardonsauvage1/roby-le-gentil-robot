#!/usr/bin/env bash
# Marqueur RViz du nid (repere d'alignement simulation <-> realite).
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec /usr/bin/python3 "$HOME/ros2_ws/tools/pc/roby_marqueur_nid.py" "$@"
