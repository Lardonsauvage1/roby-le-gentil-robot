#!/usr/bin/env bash
# Sort la tete du nid en rejouant la trajectoire teleop nettoyee (~/roby_sortie_nid.yaml).
#   ~/roby_sortie_nid.sh            # DRY : affiche la trajectoire, ne bouge pas
#   ~/roby_sortie_nid.sh --go       # BOUGE : nid -> sortie (bras AU NID, validation Sam)
#   ~/roby_sortie_nid.sh --reverse --go   # re-docker : sortie -> nid
# Prerequis : stack up (RobySystem, archi B). LE ROBOT BOUGE avec --go : surveiller.
set -e
# Neutralise pyenv (rclpy = python systeme 3.12), env ROS + DDS domaine 42
unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | paste -sd ':')"
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
exec /usr/bin/python3 "$HOME/roby_sortie_nid.py" "$@"
