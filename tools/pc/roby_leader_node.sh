#!/usr/bin/env bash
# Lance leader_node (bras guide, US-018).
#
# Pourquoi ce wrapper : le noeud a besoin d'un interpreteur ou ROS 2 ET scservo_sdk
# coexistent. Le python systeme a ROS mais pas scservo_sdk ; le venv lerobot a
# scservo_sdk mais pas ROS. La solution retenue (validee) : sourcer ROS, puis lancer
# le python du venv — ROS arrive alors par PYTHONPATH. Meme schema que
# roby_bag_to_lerobot.sh.
#
#   bash ~/roby_leader_node.sh              # materiel reel
#   bash ~/roby_leader_node.sh --sim        # aucun materiel requis
#   bash ~/roby_leader_node.sh --ids 1 2 3 4 5
#
# Le noeud REFUSE de demarrer si l'interface Ethernet declaree dans la config DDS
# n'est pas operationnelle. C'est VOULU : un repli sur le Wi-Fi ou la boucle locale
# donnerait une stack qui "marche" sans jamais parler au Pi5. On corrige le cablage,
# on ne le contourne pas.
set -e
VENV="$HOME/lerobot-experiments/venv/bin/python"
[ -x "$VENV" ] || { echo "venv lerobot introuvable : $VENV"; exit 1; }

# Reglages valides avec Sam le 2026-09-09 (teleoperation en position/cartesien).
# Surchargeables : tout -p passe en ligne de commande arrive APRES et gagne.
ROS_ARGS=(-p "ids:=[1,2,3,4,5]"
          -p "joystick_deadzone_deg:=[10.0,10.0,10.0,5.0,5.0]"
          -p "maintien_couple_pct:=35.0"
          -p "maintien_relache_deg:=0.35"
          -p "maintien_rate_hz:=25.0"
          -p "recentrage_couple_pct:=45.0"
          -p "recentrage_maintien_pct:=45.0")
while [ $# -gt 0 ]; do
  case "$1" in
    --sim|--simulate) ROS_ARGS+=(-p simulate:=true); shift ;;
    --ids) shift; L=""; while [ $# -gt 0 ] && [[ "$1" =~ ^[0-9]+$ ]]; do L="$L,$1"; shift; done
           ROS_ARGS+=(-p "ids:=[${L#,}]") ;;
    --rate) ROS_ARGS+=(-p "publish_rate_hz:=$2"); shift 2 ;;
    *) ROS_ARGS+=("$1"); shift ;;
  esac
done

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec "$VENV" -m roby_control.leader_node --ros-args "${ROS_ARGS[@]}"
