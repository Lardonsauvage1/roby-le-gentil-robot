#!/usr/bin/env bash
# Lance le noeud d'INFERENCE CARTESIENNE (roby_infer_cart.py) dans le bon env.
# Jumeau de roby_infer.sh, mais pour les modeles b2cart_* (etat = pose TCP 6D, action 7D).
#
# ⚠️ LATENCE : modele 263M => ~3.6 s d'inference pour 0.53 s d'actions.
#    Le bras BOUGE ~0.5 s puis TIENT ~3.5 s, en boucle. C'est ATTENDU (test de la chaine,
#    pas une demo de fluidite). Quand le tampon est vide, la derniere consigne est tenue.
#
# Chaine : roby_infer_cart -> DLS (IK) -> /guard/* -> roby_guard -> moteurs.
# Prerequis pour --go : garde (roby_guard.sh) + controleurs + move_group + scene,
# et la camera GAUCHE qui publie /head_camera/left/image_raw/compressed.
#
#   ~/roby_infer_cart.sh                    # DRY (infere sur vraies cameras, NE BOUGE PAS)
#   ~/roby_infer_cart.sh --go               # REEL : publie vers le garde (Sam present, coupure a portee)
#   ~/roby_infer_cart.sh --steps 5          # 2x plus rapide, qualite degradee
#   ~/roby_infer_cart.sh --model ~/deployable_models/apple/b2cart_fixed_128/pretrained_model
# Nouvel episode : ros2 service call /roby_infer/reset_episode std_srvs/srv/Trigger
set -e

VENV=/home/sam/lerobot-experiments/venv/bin/python
LEROBOT=/home/sam/lerobot-experiments
# Defaut = le PETIT cartesien (39M). Mesure epinglee sur les P-cores : 222 ms, donc
# dans le budget de 533 ms -> mouvement quasi continu. Le gros (b2cart_fixed_96, 263M)
# demande 3.6 s par inference, soit 7x le budget : le bras bouge 0.5 s puis tient 3.5 s.
# Il reste accessible via --model, mais ce n'est pas un defaut raisonnable.
MODEL_DEFAULT="$HOME/deployable_models/apple/b2cart_small_96/pretrained_model"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
export OMP_NUM_THREADS=6

cd "$LEROBOT"
exec taskset -c 0-11 "$VENV" "$HOME/roby_infer_cart.py" --model "$MODEL_DEFAULT" "$@"
