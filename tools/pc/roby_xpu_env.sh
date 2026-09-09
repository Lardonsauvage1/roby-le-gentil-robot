#!/usr/bin/env bash
# Environnement d'entrainement XPU (iGPU Arc) — a SOURCER avant tout entrainement.
#
#   source ~/roby_xpu_env.sh
#
# Pourquoi : torch.compile(backend="inductor") sur XPU a besoin de compiler des kernels
# Triton. Il lui faut les EN-TETES Level Zero et le lien .so non versionne, que le paquet
# runtime Ubuntu ne fournit pas (c'est le paquet -dev qui les a). Comme la regle du projet
# interdit apt/sudo sur atomman, on les fournit en espace utilisateur.
#
# Sans ces 2 variables : "fatal error: level_zero/ze_api.h" puis "ne peut pas trouver -lze_loader".
# Avec : torch.compile fonctionne et rapporte +41 % (109 -> 153 ech/s, mesure 2026-07-27).
#
# En-tetes recuperes depuis oneapi-src/level-zero v1.17.44 (version assortie au runtime
# installe : libze_loader.so.1.17.44).

export CPATH="$HOME/level_zero_headers:$CPATH"
export LIBRARY_PATH="$HOME/level_zero_headers/lib:$LIBRARY_PATH"

# Python d'entrainement (torch 2.8.0+xpu + IPEX 2.8.10). NE PAS confondre avec le venv de
# deploiement ~/lerobot-experiments/venv (torch CPU + OpenVINO) qui fait tourner le robot.
export ROBY_XPU_PY="$HOME/ipex_test_venv/bin/python"
