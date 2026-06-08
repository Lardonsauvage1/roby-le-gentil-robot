#!/bin/bash
# =====================================================================
# roby_2_prep.sh  —  PREPARATION (montage de la tete + dry-run)
# A executer sur le PC, APRES roby_1_init.sh.
# ATTENTION : LE BRAS BOUGE (steppers + servos + verrou).
# Sequence : init -> pose de montage -> attente 10s (tu montes la tete)
#            -> verrou -> D -> E -> A -> attente 30s -> deverrou -> init
# =====================================================================
set -u
PI=roby@192.168.1.37

echo "[prep] ATTENTION : le bras va bouger. Ctrl+C pour annuler."
sleep 2
echo "[prep] lancement de la preparation sur le Pi5..."
ssh -o ConnectTimeout=10 "$PI" 'bash -s' <<'REMOTE'
source /opt/ros/jazzy/setup.bash
source ~/rlgr/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml
python3 ~/demo_prep.py
REMOTE

echo "[prep] === PREPARATION TERMINEE ==="
