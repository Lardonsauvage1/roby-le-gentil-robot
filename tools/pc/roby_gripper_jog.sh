#!/usr/bin/env bash
# Panneau jog manuel verrou tete + pince (Python systeme 3.12 + tkinter + rclpy).
source "$(dirname "$(readlink -f "$0")")/roby_ros_env.sh" >/dev/null 2>&1
exec /usr/bin/python3 "$(dirname "$(readlink -f "$0")")/roby_gripper_jog.py" "$@"
