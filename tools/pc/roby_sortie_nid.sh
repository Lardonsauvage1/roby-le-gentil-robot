#!/usr/bin/env bash
# Sort la tete du nid en rejouant la trajectoire teleop nettoyee (~/roby_sortie_nid.yaml).
#   roby sortie [--go]            # point d'entree conseille (etat de travail pose apres)
#   ~/roby_sortie_nid.sh            # DRY : affiche la trajectoire, ne bouge pas
#   ~/roby_sortie_nid.sh --go       # BOUGE : nid -> sortie (bras AU NID, validation Sam)
#   ~/roby_sortie_nid.sh --reverse --go   # re-docker : sortie -> nid
# Prerequis : stack up (RobySystem, archi B). LE ROBOT BOUGE avec --go : surveiller.
set -e
ici="$(dirname "$(readlink -f "$0")")"
source "$ici/roby_ros_env.sh"   # domaine 42 (43 si ROBY_SIM=1), paquets de CE workspace
exec /usr/bin/python3 "$ici/roby_sortie_nid.py" "$@"
