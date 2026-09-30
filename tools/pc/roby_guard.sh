#!/usr/bin/env bash
# Lance le GARDE DU CORPS (roby_guard.py) dans le bon env ROS (comme roby_oracle_real.sh).
# Le garde filtre les consignes reseau -> moteurs (butees + vitesse + plancher + collision MoveIt).
# Il NE bouge rien tout seul : il attend des consignes sur /guard/joint_trajectory (+ /guard/gripper).
#
# Prerequis pour l'anti-collision reelle : move_group lance ET la scene chargee — c'est le cas
# apres `roby up` (pc_moveit.launch.py la charge) ; le garde refuse de demarrer sinon.
#
# Options passees en plus (ex : --max-vel 1.0 --no-floor). Voir roby_guard.py --help.
# Ctrl-C pour arreter.
set -e
ici="$(dirname "$(readlink -f "$0")")"
source "$ici/roby_ros_env.sh"   # domaine 42 (43 si ROBY_SIM=1), paquets de CE workspace
exec /usr/bin/python3 "$ici/roby_guard.py" "$@"
