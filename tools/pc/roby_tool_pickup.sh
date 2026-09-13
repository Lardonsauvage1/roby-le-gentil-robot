#!/usr/bin/env bash
# Séquence de prise d'outil via MoveIt. LE ROBOT BOUGE.
set -e
ici="$(dirname "$(readlink -f "$0")")"
source "$ici/roby_ros_env.sh"   # domaine 42 (43 si ROBY_SIM=1), paquets de CE workspace
exec /usr/bin/python3 "$ici/roby_tool_pickup.py" "$@"
