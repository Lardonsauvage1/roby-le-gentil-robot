#!/bin/bash
# Lance le noeud ROS des 2 cameras (cam_pub_pi2_dual.py = UN SEUL process : ISP partage,
# 2 process cassent le verrouillage expo/WB).
#
# GARDE par VERROU (2026-07-20) : refuse de demarrer si le noeud tourne deja. Sans elle,
# la 2e instance mourait sur "Failed to acquire camera: Device or resource busy" pendant
# qu'on croyait avoir redemarre les cameras. On utilise flock (atomique) et PAS un grep
# sur ps : un grep matche la ligne de commande de qui l'appelle (auto-match, vecu 2x).
#
# LOG EN AJOUT (>>) : un ">" tronquait le log du noeud VIVANT ; son descripteur de fichier
# suit l'inode, donc ses vraies traces continuaient dans l'ancien fichier pendant que le
# nom canonique ne contenait plus que l'erreur d'un process mort. Piege a diagnostic vecu.
set -e

LOG=/home/roby/dual_node.log
LOCK=/home/roby/.cam_pub_dual.lock

# Garde 1 : process deja vivant. Attrape AUSSI les instances lancees par l'ancien
# script (qui ne detiennent pas le verrou). `pgrep -f` ne s'auto-matche pas ici :
# la ligne de commande de ce script est "bash launch_cams.sh", pas le motif cherche.
RUNNING=$(pgrep -f 'cam_pub_pi2_dual\.py' | grep -v "^$$\$" || true)
if [ -n "$RUNNING" ]; then
  echo "❌ Les cameras tournent DEJA (PID $RUNNING) - rien a faire." >&2
  # PAS de -e ici : il signifie "tous les processus" et annulerait le -p.
  ps -o pid,etime,args -p $RUNNING 2>/dev/null | tail -n +2 >&2
  echo "   Pour redemarrer : kill -9 $RUNNING, puis relancer." >&2
  exit 1
fi

# Garde 2 : verrou atomique (course entre 2 lancements simultanes).
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "❌ Un autre lancement est en cours (verrou $LOCK tenu)." >&2
  exit 1
fi

LC=/home/roby/lc_src/libcamera-0.5.2+rpt20250903/build/src/py
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=/home/roby/pystubs:$LC:$PYTHONPATH

echo "===== demarrage $(date -Is) =====" >> "$LOG"
python3 /home/roby/cam_pub_pi2_dual.py >> "$LOG" 2>&1 &
CAM_PID=$!
echo "cameras lancees (PID $CAM_PID), log : $LOG"
wait "$CAM_PID"
