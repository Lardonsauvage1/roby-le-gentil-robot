#!/usr/bin/env bash
# Panneau de surveillance de la consigne PINCE du modele (lecture seule, ne commande RIEN).
# Montre la valeur brute continue predite, le seuil 0.5, la courbe 60 s et les bascules.
# Marche aussi quand roby_infer_cart tourne en DRY (sans --go) : on observe sans bouger.
#
#   bash ~/roby_gripper_monitor.sh
set -e

# rclpy = python systeme 3.12 (pas le venv lerobot, pas pyenv)
unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | grep -v '/venv/' | paste -sd ':')"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
export DISPLAY="${DISPLAY:-:0}"

exec /usr/bin/python3 "$HOME/roby_gripper_monitor.py"
