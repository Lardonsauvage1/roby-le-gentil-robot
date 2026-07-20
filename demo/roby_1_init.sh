#!/bin/bash
# =====================================================================
# roby_1_init.sh  —  INIT COMPLET (a lancer juste apres avoir rallume)
# A executer sur le PC. Monte sur le Pi5 :
#   - la stack bras (~/rlgr : ros2_control + move_group)
#   - le noeud verrou de tete (/head_lock)
#   - le noeud pince (/gripper)
# AUCUN mouvement du bras (les steppers tiennent leur position).
# Les servos verrou/pince se mettent au repos : deverrou (50) / pince ouverte (120).
# =====================================================================
set -u
PI=roby@192.168.1.37

echo "[init] connexion au Pi5 + montage de la stack..."
ssh -o ConnectTimeout=10 "$PI" 'bash -s' <<'REMOTE'
source /opt/ros/jazzy/setup.bash
source ~/rlgr/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml

# 0) Nettoyage d eventuels restes (idempotent)
for pat in "[r]os2_control_node" "[m]ove_group" "[r]obot_state_pub" "[s]pawner" "[h]ead_lock_node" "[g]ripper_node"; do
  for P in $(pgrep -f "$pat"); do kill -9 "$P" 2>/dev/null; done
done
sleep 2

# 1) Stack bras  (redirection AU NIVEAU setsid pour detacher du canal ssh)
echo "[init] lancement stack bras..."
rm -f /tmp/robot_full.log
setsid bash -c "ros2 launch roby_hardware robot_full.launch.py" > /tmp/robot_full.log 2>&1 < /dev/null &
printf "[init] attente des controleurs"
for i in $(seq 1 40); do
  if [ "$(grep -c 'Successfully switched' /tmp/robot_full.log 2>/dev/null)" -ge 2 ]; then break; fi
  printf "."; sleep 1
done
echo
grep -h "Initialized with" /tmp/robot_full.log | tail -1

# 2) Noeud verrou tete
echo "[init] noeud verrou..."
rm -f /tmp/head_lock.log
setsid bash -c "python3 ~/head_lock_node.py" > /tmp/head_lock.log 2>&1 < /dev/null &

# 3) Noeud pince
echo "[init] noeud pince..."
rm -f /tmp/gripper.log
setsid bash -c "python3 ~/gripper_node.py" > /tmp/gripper.log 2>&1 < /dev/null &
sleep 4

echo "[init] etat servos :"
tail -1 /tmp/head_lock.log
tail -1 /tmp/gripper.log
REMOTE

echo "[init] ============================================"
echo "[init]  ROBOT PRET (stack + verrou + pince)."
echo "[init]  Aucun mouvement effectue. Prochaine etape :"
echo "[init]    ./roby_2_prep.sh   (preparation / montage tete)"
echo "[init]    ./roby_3_demo.sh   (demo complete + IHM)"
echo "[init] ============================================"
