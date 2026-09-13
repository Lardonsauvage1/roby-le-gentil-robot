#!/usr/bin/env bash
# Wrapper d'entrainement XPU : env Level Zero (torch.compile) + ROS (lecture dataset) + venv IPEX.
set -e
source "$HOME/roby_xpu_env.sh"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
exec "$HOME/ipex_test_venv/bin/python" "$HOME/ros2_ws/tools/pc/roby_train_xpu.py" "$@"
