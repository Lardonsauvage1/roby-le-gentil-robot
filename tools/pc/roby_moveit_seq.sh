#!/usr/bin/env bash
# Exécute une séquence de poses via MoveIt (anti-collision). Ex: roby_moveit_seq.sh nid A B
set -e
ici="$(dirname "$(readlink -f "$0")")"
source "$ici/roby_ros_env.sh"   # domaine 42 (43 si ROBY_SIM=1), paquets de CE workspace
exec /usr/bin/python3 "$ici/roby_moveit_seq.py" "$@"
