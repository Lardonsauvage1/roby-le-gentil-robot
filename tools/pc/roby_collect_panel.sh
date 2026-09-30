#!/usr/bin/env bash
# Lance le panneau de collecte dataset (tri 1-par-1) sur le PC (point d'entree : `roby collecte`).
# Python SYSTEME (3.12, ROS/tkinter/rclpy), PAS pyenv : cf. roby_ros_env.sh.
set -e
ici="$(dirname "$(readlink -f "$0")")"
source "$ici/roby_ros_env.sh"   # domaine 42, paquets de CE workspace
exec /usr/bin/python3 "$ici/roby_collect_panel.py"
