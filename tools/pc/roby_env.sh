# A SOURCER dans un terminal PC :  source ~/roby_env.sh   (ROBY_SIM=1 avant : simulation, domaine 43)
# Delegue a roby_ros_env.sh, la seule copie de l'environnement ROS du PC.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/roby_ros_env.sh" || return 1
echo "[roby_env] ROS Jazzy, domaine $ROS_DOMAIN_ID ($([ "$ROBY_SIM" = 1 ] && echo simulation || echo 'vrai robot, enp86s0 -> Pi5')), workspace $ROBY_WS."
