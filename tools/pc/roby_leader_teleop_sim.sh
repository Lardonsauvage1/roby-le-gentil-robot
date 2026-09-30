#!/usr/bin/env bash
# Teleoperation du bras SIMULE par le bras guide (US-020).
# N'envoie RIEN au vrai robot : anime seulement le modele pour RViz.
#   bash ~/roby_leader_teleop_sim.sh [--vitesse 0.6]
set -e
ROS_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --vitesse) ROS_ARGS+=(-p "vitesse_max_rad_s:=$2"); shift 2 ;;
    *) ROS_ARGS+=("$1"); shift ;;
  esac
done
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec /usr/bin/python3 -m roby_control.leader_teleop_sim --ros-args "${ROS_ARGS[@]}"
