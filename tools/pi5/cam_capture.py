#!/usr/bin/env python3
"""Capture UNE camera (index passe en arg) et ecrit la derniere frame JPEG dans
/dev/shm de facon atomique. Lance par cam_switch.py en sous-process dedie : un
process FRAIS par camera (l'ISP PiSP ne supporte pas la reouverture in-process).
Usage : env LD_PRELOAD=<v4l2-compat.so> python3 cam_capture.py <idx_opencv> <nom>
"""
import cv2, sys, time, os, signal

idx = int(sys.argv[1])
name = sys.argv[2] if len(sys.argv) > 2 else str(idx)
OUT = "/dev/shm/roby_cam.jpg"
FPS = 15

_run = {'on': True}


def _stop(*a):
    # Liberer PROPREMENT la camera/ISP PiSP partage avant de mourir, sinon la
    # camera suivante ne peut pas l'acquerir (handoff casse).
    _run['on'] = False


signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT, _stop)

cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while _run['on']:
    ret, f = cap.read()
    if ret and f is not None and float(f.mean()) > 0.5:
        cv2.putText(f, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0), 2)
        ok, buf = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            tmp = OUT + ".tmp"
            with open(tmp, 'wb') as fh:
                fh.write(buf.tobytes())
            os.replace(tmp, OUT)
    time.sleep(1.0 / FPS)

cap.release()  # liberation propre de l'ISP partage (sur SIGTERM/SIGINT)

