#!/usr/bin/env bash
# Attribution des IDs + banc du bus chaine du bras guide (US-017).
# ECRIT le registre ID (et lui seul), sous garde-fous. Dry-run par defaut.
#
#   bash ~/roby_leader_setup.sh set-id --to 1          # dry-run
#   bash ~/roby_leader_setup.sh set-id --to 1 --go     # ecrit
#   bash ~/roby_leader_setup.sh syncread --duration 60
#   bash ~/roby_leader_setup.sh current
set -e
VENV="$HOME/lerobot-experiments/venv/bin/python"
[ -x "$VENV" ] || { echo "venv lerobot introuvable : $VENV"; exit 1; }
exec "$VENV" "$HOME/ros2_ws/tools/pc/roby_leader_setup.py" "$@"
