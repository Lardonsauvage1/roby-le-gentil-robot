#!/usr/bin/env bash
# Lance l'outil de jog fin / calibration nid sur le PC (point d'entree conseille : `roby jog`).
# Python SYSTEME (3.12, ROS) et NON pyenv (3.11 sans tkinter/rclpy) : cf. roby_ros_env.sh.
set -e
ici="$(dirname "$(readlink -f "$0")")"
source "$ici/roby_ros_env.sh"   # domaine 42 (43 si ROBY_SIM=1), paquets de CE workspace
exec /usr/bin/python3 "$ici/roby_fine_jog.py"
