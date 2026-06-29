#!/usr/bin/env bash
# Séquence de prise d'outil via MoveIt. LE ROBOT BOUGE.
source "$HOME/roby_env.sh" >/dev/null 2>&1
exec /usr/bin/python3 "$HOME/roby_tool_pickup.py" "$@"
