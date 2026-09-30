#!/usr/bin/env bash
# Teleoperation en POSITION du bras SIMULE par le bras guide.
# Le guide est une maquette : sa pose devient celle de Roby. N'envoie RIEN au vrai robot.
#   bash ~/roby_leader_teleop_pos.sh [-p lissage:=0.3]
# Protection BUG-008 dans le noeud : il se tait s'il voit un autre publisher de
# /joint_states (vrai robot). Pour simuler a cote de la vraie stack : ROS_DOMAIN_ID=43.
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec /usr/bin/python3 -m roby_control.leader_teleop_pos --ros-args "$@"
