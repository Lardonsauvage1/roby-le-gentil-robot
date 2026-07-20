#!/usr/bin/env bash
# Lance le noeud d'INFERENCE (roby_infer.py) dans le bon env :
#   - venv lerobot  (torch + lerobot + cv2)   -> /home/sam/lerobot-experiments/venv/bin/python
#   - ROS jazzy      (rclpy + messages)        -> source /opt/ros + workspace
#   - EPINGLE sur les 6 P-cores (taskset -c 0-11) + 6 threads  -> ~230 ms/inference @50 pas
#     (sinon les E-cores plombent : ~730 ms). CRITIQUE pour tenir la cadence.
#
# Chaine : roby_infer -> /guard/* -> roby_guard -> moteurs.
# Prerequis pour --go : garde (roby_guard.sh) + controleurs + move_group + scene charges,
# et les 2 cameras qui publient /head_camera/{left,right}/image_raw/compressed.
#
#   ~/roby_infer.sh                         # DRY (mesure latence sur vraies cameras, NE BOUGE PAS)
#   ~/roby_infer.sh --go                    # REEL : publie vers le garde (Sam present, coupure a portee)
#   ~/roby_infer.sh --model <chemin> --go   # autre modele (defaut = A_cd5k_from2000/brut)
#   ~/roby_infer.sh --steps-max 32          # plafond de debruitage plus bas
# Nouvel episode : ros2 service call /roby_infer/reset_episode std_srvs/srv/Trigger
set -e

VENV=/home/sam/lerobot-experiments/venv/bin/python
LEROBOT=/home/sam/lerobot-experiments
MODEL_DEFAULT="$HOME/deployable_models/apple/b2_fixed_96/pretrained_model"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
export OMP_NUM_THREADS=6

cd "$LEROBOT"   # chemins de modele relatifs pratiques
# --model par defaut d'abord, "$@" ensuite (argparse : la derniere valeur gagne si l'utilisateur en passe une)
exec taskset -c 0-11 "$VENV" "$HOME/roby_infer.py" --model "$MODEL_DEFAULT" "$@"
