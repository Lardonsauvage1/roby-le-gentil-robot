#!/usr/bin/env python3
"""cam_pub_pi2.py — Publieur camera via PICAMERA2 avec exposition/gain/WB FIGES.

Objectif : rendu REPRODUCTIBLE d'une session/reboot a l'autre (crucial pour le
reseau de neurones). Contrairement a l'ancien cam_pub.py (cv2 + shim v4l2-compat,
qui laisse l'auto-expo/AWB de libcamera deriver), ici on VERROUILLE au niveau capteur :
AeEnable=False, AwbEnable=False, + ExposureTime/AnalogueGain/ColourGains en dur.

Selection camera par sous-chaine d'Id (i2c) => deterministe, pas d'ambiguite d'index :
  gauche (exterieure) = i2c@80000 ; droite (poignet) = i2c@88000.

Prerequis env (le module libcamera py3.12 est compile hors des chemins standard) :
  LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu   (=> libcamera systeme 0.5.2, PiSP)
  PYTHONPATH=~/pystubs:~/lc_src/libcamera-0.5.2+rpt20250903/build/src/py
Publie sensor_msgs/CompressedImage (JPEG) sur /head_camera/<side>/image_raw/compressed.
"""
import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2


def pick_camera(id_substr):
    for i, info in enumerate(Picamera2.global_camera_info()):
        if id_substr in info.get("Id", ""):
            return i
    raise RuntimeError(f"camera contenant '{id_substr}' introuvable")


class CamPub(Node):
    def __init__(self, args):
        super().__init__(f"cam_pub_{args.side}")
        topic = f"/head_camera/{args.side}/image_raw/compressed"
        self.pub = self.create_publisher(CompressedImage, topic, qos_profile_sensor_data)
        self.args = args
        self.rot = args.rot180
        self.enc = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
        self.frame_id = f"head_camera_{args.side}"

        idx = pick_camera(args.cam)
        self.cam = Picamera2(idx)
        # Réplique EXACTE du standalone qui verrouille : still-config + set_controls APRES start.
        cfg = self.cam.create_still_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"})
        self.cam.configure(cfg)
        self.cam.start()
        fd = int(1e6 / args.fps)
        # Contrôles VERROUILLÉS. RÉ-ASSERTÉS périodiquement dans _loop : avec 2 caméras sur
        # l'ISP PiSP partagé, le démarrage de la 2e casse le lock de la 1re -> on le ré-impose.
        self.locked = {
            "AeEnable": False, "AwbEnable": False,
            "ExposureTime": args.exposure,
            "AnalogueGain": args.gain,
            "ColourGains": (args.red_gain, args.blue_gain),
            "FrameDurationLimits": (fd, fd),
        }
        self.cam.set_controls(self.locked)
        time.sleep(1.5)   # laisse les controles s'appliquer (comme le standalone)
        self.get_logger().info(
            f"cam '{args.side}' idx={idx} ({args.cam}) -> {topic}  "
            f"{args.width}x{args.height} @{args.fps}fps  FIGE exp={args.exposure} "
            f"gain={args.gain} cg=({args.red_gain},{args.blue_gain}) rot180={self.rot}")
        self._n = 0; self._bytes = 0; self._t0 = time.monotonic()
        # Capture dans un THREAD DÉDIÉ (pas le timer rclpy) : toutes les opérations
        # picamera2 (set_controls + capture) dans le MÊME thread => le verrouillage
        # des contrôles s'applique (le timer rclpy, executor, le cassait).
        self.period = 1.0 / args.fps
        self._running = True
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def _loop(self):
        k = 0
        while self._running and rclpy.ok():
            t0 = time.monotonic()
            k += 1
            if k % 15 == 0:                      # ré-assertion ~1x/s (défend le lock vs concurrence)
                self.cam.set_controls(self.locked)
            frame = self.cam.capture_array("main")   # RGB888 (== BGR ordre OpenCV ici)
            if self.rot:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            ok, jpg = cv2.imencode(".jpg", frame, self.enc)
            if ok:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.format = "jpeg"
                msg.data = jpg.tobytes()
                self.pub.publish(msg)
                self._n += 1; self._bytes += len(msg.data)
            dt = time.monotonic() - self._t0
            if dt >= 5.0:
                self.get_logger().info(
                    f"[stats] {self._n/dt:.1f} fps | {self._bytes/max(1,self._n)/1024:.0f} KB/frame")
                self._n = 0; self._bytes = 0; self._t0 = time.monotonic()
            sl = self.period - (time.monotonic() - t0)
            if sl > 0:
                time.sleep(sl)

    def destroy_node(self):
        self._running = False
        try:
            self._thr.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.cam.stop(); self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--side", required=True, help="left|right (nom du topic)")
    p.add_argument("--cam", required=True, help="sous-chaine Id camera (ex: i2c@80000)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--quality", type=int, default=80)
    p.add_argument("--rot180", action="store_true")
    p.add_argument("--exposure", type=int, required=True, help="ExposureTime us (FIGE)")
    p.add_argument("--gain", type=float, required=True, help="AnalogueGain (FIGE)")
    p.add_argument("--red-gain", type=float, required=True, help="ColourGains rouge (FIGE)")
    p.add_argument("--blue-gain", type=float, required=True, help="ColourGains bleu (FIGE)")
    args = p.parse_args()
    rclpy.init()
    node = CamPub(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
