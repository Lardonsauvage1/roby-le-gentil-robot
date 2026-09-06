#!/usr/bin/env bash
# Affichage direct bruts <-> convertis, pour regler la calibration leader -> Roby (US-019).
# Lecture seule : ne commande rien. Necessite leader_node en marche.
#   bash ~/roby_leader_calib.sh [--hz 5] [--calib <fichier>]
set -e
VENV="$HOME/lerobot-experiments/venv/bin/python"
[ -x "$VENV" ] || { echo "venv lerobot introuvable : $VENV"; exit 1; }
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec "$VENV" -m roby_control.leader_calib_live "$@"
