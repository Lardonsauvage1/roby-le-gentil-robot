#!/usr/bin/env bash
# Banc LECTURE SEULE du bras guide Feetech STS3215 (US-016).
# N'ecrit JAMAIS dans un servo : ni couple, ni ID, ni baudrate.
#
#   bash ~/roby_leader_bench.sh scan
#   bash ~/roby_leader_bench.sh scan --all-bauds
#   bash ~/roby_leader_bench.sh read
set -e

# scservo_sdk vit dans le venv lerobot (pas de ROS necessaire ici)
VENV="$HOME/lerobot-experiments/venv/bin/python"
[ -x "$VENV" ] || { echo "venv lerobot introuvable : $VENV"; exit 1; }

exec "$VENV" "$(dirname "$(readlink -f "$0")")/roby_leader_bench.py" "$@"
