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
source "$HOME/roby_env.sh" >/dev/null 2>&1

TOPICS=(
  /head_camera/left/image_raw/compressed
  /head_camera/right/image_raw/compressed
  /joint_states
  /roby_infer/action /roby_infer/gripper_raw /roby_infer/status
  /guard/joint_trajectory /guard/gripper
)

# Motifs en [x]yz : pgrep -f ne doit pas trouver ce script lui-meme.
actif() { pgrep -f "[r]oby_guard.py|[r]oby_infer_cart.py .*--go" >/dev/null; }

echo "veilleur pret : $(date '+%F %T') -> $OUT_ROOT"
while true; do
  until actif; do sleep 0.2; done
  BAG="$OUT_ROOT/rollout_$(date +%Y%m%d_%H%M%S)"
  echo "$(date '+%T') DEMARRER detecte -> $BAG"
  taskset -c 12-19 ros2 bag record -s mcap -o "$BAG" "${TOPICS[@]}" > "$BAG.log" 2>&1 &
  REC=$!
  MODEL=""
  while actif; do
    # le panneau lance le modele 4 s APRES le garde : on relit tant qu'il manque
    [ -z "$MODEL" ] && MODEL=$(pgrep -af "[r]oby_infer_cart.py" | grep -oE -- "--model [^ ]+" | head -1)
    sleep 0.2
  done
  kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null
  echo "${MODEL:-modele inconnu (aucun roby_infer_cart vu pendant ce test)}" > "$BAG.info"
  echo "$(date '+%T') STOP detecte -> bag ferme : $BAG"
done
