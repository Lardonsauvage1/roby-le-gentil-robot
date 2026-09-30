#!/usr/bin/env bash
# Teleoperation CARTESIENNE a echelle variable du bras SIMULE (US-026).
# N'envoie RIEN au vrai robot.
#   bash ~/roby_leader_teleop_cart.sh [-p echelle:=0.2]
# Protection BUG-008 dans le noeud : il se tait s'il voit un autre publisher de
# /joint_states (vrai robot). Pour simuler a cote de la vraie stack : ROS_DOMAIN_ID=43.
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec /usr/bin/python3 -m roby_control.leader_teleop_cart --ros-args "$@"
