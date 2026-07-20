#!/usr/bin/env bash
# Lance la boucle ORACLE en mode REEL (le bras BOUGE). Defaut : 1 episode,
# libre=0.30, ligne_droite=0.02 m/s. Options passees en plus (ecrasent les defauts) :
#   ~/roby_oracle_real.sh                              # 1 episode, vitesses par defaut
#   ~/roby_oracle_real.sh --vel 0.4 --cart-speed 0.015 # ajuste les vitesses
#   ~/roby_oracle_real.sh --episodes 3                 # 3 episodes
# LE ROBOT BOUGE — surveiller, doigt sur la coupure moteurs. Ctrl-C pour arreter le script.
set -e

# Neutralise pyenv (rclpy = python systeme 3.12)
unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | paste -sd ':')"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"

exec /usr/bin/python3 "$HOME/roby_oracle.py" --mode real --go --episodes 1 "$@"
