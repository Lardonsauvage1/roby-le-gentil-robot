#!/usr/bin/env bash
# Teleoperation du VRAI bras par le bras guide (US-023, ADR-003 option B).
#
#   bash ~/roby_leader_teleop_reel.sh [-p echelle:=0.3] [...]
#
# Le noeud envoie ses consignes au GARDE (/guard/joint_trajectory), jamais directement
# au controleur, et ne publie rien sur /joint_states. Il demarre DEBRAYE.
#
# Le garde : si aucun roby_guard ne tourne, ce script en lance un avec la MEME
# compensation joint_3 que le noeud (sinon ses verifications plancher/collision
# porteraient sur une pose fausse de ~3 cm) et l'arrete en sortant. S'il en tourne
# deja un (lance par le panneau du modele), il est reutilise si sa compensation est
# la meme, sinon refus.
#
# Prerequis : stack lancee (/roby-lancer-bras), move_group + scene chargee, leader_node
# lance (~/roby_leader_node.sh), panneau du modele ARRETE (un seul emetteur vers le garde).
#
# Premier essai (US-023) : echelle 0.5, vitesse plafonnee a 0.05 m/s -- les defauts.
#
# Le bras SIMULE reste pilote par ~/roby_leader_teleop_cart.sh, inchange.
set -e

unset PYENV_VERSION 2>/dev/null || true
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.pyenv' | grep -v '/venv/' | paste -sd ':')"
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/cyclone_config.xml}"
# Meme valeur que le panneau du modele (roby_infer_panel.py).
export ROBY_J3_SCALE="${ROBY_J3_SCALE:-0.9299}"

GARDE_PY="$HOME/ros2_ws/tools/pc/roby_guard.py"
GARDE_PID=""
existant=$(pgrep -f "^[^ ]*python[^ ]* [^ ]*roby_guard\.py" | head -1 || true)
if [ -n "$existant" ]; then
    sa=$(tr '\0' '\n' < /proc/"$existant"/environ | sed -n 's/^ROBY_J3_SCALE=//p')
    dom=$(tr '\0' '\n' < /proc/"$existant"/environ | sed -n 's/^ROS_DOMAIN_ID=//p')
    if [ "${sa:-0}" != "$ROBY_J3_SCALE" ] || [ "${dom:-0}" != "$ROS_DOMAIN_ID" ]; then
        echo "REFUS : un garde tourne deja (PID $existant) avec ROBY_J3_SCALE=${sa:-absent}," \
             "ROS_DOMAIN_ID=${dom:-absent} ; attendu $ROBY_J3_SCALE, $ROS_DOMAIN_ID." >&2
        exit 1
    fi
    echo "garde existant reutilise (PID $existant, ROBY_J3_SCALE=$sa)"
else
    /usr/bin/python3 "$GARDE_PY" > /tmp/guard_teleop.log 2>&1 &
    GARDE_PID=$!
    echo "garde lance (PID $GARDE_PID, journal /tmp/guard_teleop.log, ROBY_J3_SCALE=$ROBY_J3_SCALE)"
fi

# SIGTERM et pas SIGINT : un processus lance en arriere-plan par un script ignore
# SIGINT (constate le 2026-09-13 avec l'enregistreur).
arreter() {
    if [ -n "$GARDE_PID" ] && kill -0 "$GARDE_PID" 2>/dev/null; then
        kill -TERM "$GARDE_PID" 2>/dev/null || true
        wait "$GARDE_PID" 2>/dev/null || true
        echo "garde arrete"
    fi
}
trap arreter EXIT

# Au PREMIER PLAN : Ctrl-C atteint le noeud, qui debraye et arrete le bras avant de sortir.
/usr/bin/python3 -m roby_control.leader_teleop_reel --ros-args -p echelle:=0.5 "$@"
