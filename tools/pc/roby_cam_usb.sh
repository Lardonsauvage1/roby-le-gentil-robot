#!/usr/bin/env bash
# Réglage + placement de la caméra USB OV4689 (« AK-Camera ») -> http://localhost:8090/
# Besoin : Linux (V4L2), Python 3 avec opencv-python et numpy.
# Python utilisé : $ROBY_CAM_PY s'il est défini, sinon le venv ipex de l'AtomMan s'il existe,
# sinon python3.
set -e
PY="${ROBY_CAM_PY:-$HOME/ipex_test_venv/bin/python}"
[ -x "$PY" ] || PY="$(command -v python3)"
"$PY" -c "import cv2, numpy" 2>/dev/null || {
    echo "Il faut OpenCV et numpy pour $PY :  $PY -m pip install opencv-python numpy" >&2; exit 1; }
exec "$PY" -u "$(dirname "$(readlink -f "$0")")/roby_cam_usb.py" "$@"
