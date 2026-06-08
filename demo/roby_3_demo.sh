#!/bin/bash
# =====================================================================
# roby_3_demo.sh  —  DEMO COMPLETE + IHM (RViz)
# A executer sur le PC, APRES roby_1_init.sh.
# 1) Ouvre l IHM (RViz MoveIt) sur le PC si pas deja ouverte.
# 2) Lance la demo sur le Pi5.
# ATTENTION : LE BRAS BOUGE (mouvements + verrou + prise/depose pince).
# Sequence : init -> A -> verrou -> B -> pince -> C -> D -> prise
#            -> E -> depose -> A -> deverrou -> init
# =====================================================================
PI=roby@192.168.1.37

# --- 1) IHM (RViz) sur le PC ---
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/sam/cyclone_config.xml
export DISPLAY=:0
unset GTK_PATH   # sinon snap VS Code casse rviz2 (BUG-003)

if pgrep -f "lib/rviz2/rviz2" >/dev/null; then
  echo "[demo] IHM (RViz) deja ouverte."
else
  echo "[demo] ouverture de l IHM (RViz)..."
  ( source /opt/ros/jazzy/setup.bash
    source /home/sam/ros2_ws/install/setup.bash
    setsid env -u GTK_PATH ros2 launch neuroneimitationcarote_moveit_config rviz_only.launch.py \
      > /tmp/rviz.log 2>&1 < /dev/null & )
  echo "[demo] attente de RViz..."
  sleep 12
fi

# --- 2) Demo sur le Pi5 ---
echo "[demo] ATTENTION : le bras va bouger. Ctrl+C pour annuler."
sleep 2
echo "[demo] lancement de la demo sur le Pi5..."
ssh -o ConnectTimeout=10 "$PI" 'bash -s' <<'REMOTE'
source /opt/ros/jazzy/setup.bash
source ~/rlgr/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml
python3 ~/demo_orchestrator.py
REMOTE

echo "[demo] === DEMO TERMINEE ==="
