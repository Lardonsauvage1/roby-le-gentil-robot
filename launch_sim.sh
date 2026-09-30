#!/bin/bash
# PERIME (2026-09-13) — renvoie a `roby up --sim`.
#
# Ce script lancait la demo MoveIt (demo.launch.py) : robot simule sans scene de collision,
# sans le launch du vrai robot, et jusqu'au 2026-09-13 sur le domaine 42 du vrai bras.
# La simulation passe maintenant par le MEME chemin que le vrai robot (robot_control.launch.py
# use_mock:=true + pc_moveit.launch.py + scene cuisine), sur le domaine 43 :
#
#     tools/pc/roby up --sim        puis  roby sortie --sim --go, roby jog --sim, roby down --sim
echo "launch_sim.sh est perime : lancement de \`roby up --sim\` (meme chaine que le vrai robot, domaine 43)." >&2
exec "$(dirname "$(readlink -f "$0")")/tools/pc/roby" up --sim "$@"
