#!/usr/bin/env bash
# Latence d'inference CPU. Utilise le venv de DEPLOIEMENT (torch CPU + OpenVINO),
# celui qui fait vraiment tourner le robot -- pas le venv XPU d'entrainement.
set -e
exec "$HOME/lerobot-experiments/venv/bin/python" "$(dirname "$(readlink -f "$0")")/roby_bench_latence.py" "$@"
