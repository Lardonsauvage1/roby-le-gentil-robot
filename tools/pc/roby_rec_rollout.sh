#!/bin/bash
# Enregistre les 2 cameras pendant chaque essai du modele lance depuis roby_infer_panel.py.
#
# Veilleur : un bag MCAP par essai, du clic sur DEMARRER (le panneau lance d'abord le
# garde roby_guard.py) jusqu'a l'arret du modele ET du garde (bouton STOP).
# Lecture seule : n'envoie rien au bras.
#
# Tourne sur les E-cores (12-19) : l'inference est sur les P-cores (0-11) + iGPU, il ne
# faut pas lui voler de temps (budget 0,53 s, cf roby_infer_panel.py).
#
#   bash ~/roby_rec_rollout.sh [dossier]          # defaut ~/roby_datasets/rollouts
#   pkill -f roby_rec_rollout.sh                   # arreter le veilleur
#
# Topics : les 2 cameras (demande de Sam) + ce qu'il faut pour relire l'essai :
# /joint_states, sorties du modele (/roby_infer/action, /roby_infer/gripper_raw,
# /roby_infer/status) et consignes vers le garde (/guard/joint_trajectory, /guard/gripper).

OUT_ROOT="${1:-$HOME/roby_datasets/rollouts}"
mkdir -p "$OUT_ROOT"
source "$(dirname "$(readlink -f "$0")")/roby_ros_env.sh" >/dev/null 2>&1

TOPICS=(
  /head_camera/left/image_raw/compressed
  /head_camera/right/image_raw/compressed
  /joint_states
  /roby_infer/action /roby_infer/gripper_raw /roby_infer/status
  /guard/joint_trajectory /guard/gripper
)

# Seuls de VRAIS processus Python executant ces scripts comptent (motif ancre en debut
# de ligne de commande). Un motif libre "roby_guard.py" declenchait aussi sur tout
# processus citant ce nom (editeur, shell de test...) : vecu le 2026-09-13, un bag
# parasite de 99 s bras immobile.
# Seul le MODELE compte (plus le garde) : le garde tourne aussi pendant la teleoperation
# du vrai bras (roby_leader_teleop_reel.sh), qui etait alors enregistree comme un essai
# « modele inconnu » (revue du 2026-09-13).
actif() {
  pgrep -f "^[^ ]*python[^ ]* [^ ]*roby_infer_cart\.py .*--go" >/dev/null
}

echo "veilleur pret : $(date '+%F %T') -> $OUT_ROOT"
while true; do
  until actif; do sleep 0.2; done
  BAG="$OUT_ROOT/rollout_$(date +%Y%m%d_%H%M%S)"
  echo "$(date '+%T') DEMARRER detecte -> $BAG"
  taskset -c 12-19 ros2 bag record -s mcap -o "$BAG" --topics "${TOPICS[@]}" > "$BAG.log" 2>&1 &
  REC=$!
  MODEL=""
  while actif; do
    # le panneau lance le modele 4 s APRES le garde : on relit tant qu'il manque
    [ -z "$MODEL" ] && MODEL=$(pgrep -af "[r]oby_infer_cart.py" | grep -oE -- "--model [^ ]+" | head -1)
    sleep 0.2
  done
  # SIGTERM et PAS SIGINT : lance en arriere-plan par un script (sans controle de
  # taches), l'enregistreur HERITE de SIGINT ignore -> kill -INT ne fait rien et le bag
  # continue indefiniment (vecu le 2026-09-13 : 2 min 50 enregistrees apres STOP).
  # SIGTERM est gere par rosbag2 : fermeture propre, metadata.yaml ecrit.
  kill -TERM "$REC" 2>/dev/null
  for _ in $(seq 1 50); do kill -0 "$REC" 2>/dev/null || break; sleep 0.2; done
  if kill -0 "$REC" 2>/dev/null; then
    kill -KILL "$REC" 2>/dev/null
    echo "$(date '+%T') ATTENTION : enregistreur tue apres 10 s (bag a reindexer : ros2 bag reindex)"
  fi
  wait "$REC" 2>/dev/null
  echo "${MODEL:-modele inconnu (aucun roby_infer_cart vu pendant ce test)}" > "$BAG.info"
  echo "$(date '+%T') STOP detecte -> bag ferme : $BAG"
done
