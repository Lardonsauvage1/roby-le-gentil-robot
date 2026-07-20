#!/usr/bin/env bash
# Lance le GARDE DU CORPS (roby_guard.py) dans le bon env ROS (comme roby_oracle_real.sh).
# Le garde filtre les consignes reseau -> moteurs (butees + vitesse + plancher + collision MoveIt).
# Il NE bouge rien tout seul : il attend des consignes sur /guard/joint_trajectory (+ /guard/gripper).
#
# Prerequis pour l'anti-collision reelle : move_group lance ET la scene chargee :
#     ros2 run roby_environments scene_loader --env atelier_actuel
#
# Options passees en plus (ex : --max-vel 1.0 --no-floor). Voir roby_guard.py --help.
# Ctrl-C pour arreter.
set -e

# Neutralise pyenv (rclpy = python systeme 3.12)
unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | grep -v '/venv/' | paste -sd ':')"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"

exec /usr/bin/python3 "$HOME/roby_guard.py" "$@"
