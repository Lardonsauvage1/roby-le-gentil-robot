#!/usr/bin/env bash
# Exécute une séquence de poses via MoveIt (anti-collision).
# Force le Python système + ROS + DDS. Ex: roby_moveit_seq.sh nid A B
source "$HOME/roby_env.sh" >/dev/null 2>&1
exec /usr/bin/python3 "$HOME/roby_moveit_seq.py" "$@"
