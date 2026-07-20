#!/usr/bin/env bash
# Panneau jog manuel verrou tete + pince (Python systeme 3.12 + tkinter + rclpy).
source "$HOME/roby_env.sh" >/dev/null 2>&1
exec /usr/bin/python3 "$HOME/roby_gripper_jog.py" "$@"
