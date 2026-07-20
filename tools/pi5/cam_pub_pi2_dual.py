#!/usr/bin/env python3
"""cam_pub_pi2_dual.py — Les 2 caméras picamera2 dans UN SEUL process (CameraManager
partagé). Necessaire : 2 process separes cassent le verrouillage manuel (concurrence ISP).
Contrôles exposition/gain/WB FIGES + re-assertes => rendu reproductible."""
import threading
import time

import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2

# Valeurs FIGEES par cote (a re-tuner au besoin). left=EXTERIEURE i2c@88000, right=POIGNET i2c@80000.
CAMS = [
    dict(side="left",  cam="i2c@88000", rot180=True,  exposure=66640, gain=6.875, red=1.039, blue=1.616),
    dict(side="right", cam="i2c@80000", rot180=False, exposure=66640, gain=8.0,   red=1.25,  blue=2.4),
]


def pick(id_substr):
    for i, info in enumerate(Picamera2.global_camera_info()):
        if id_substr in info.get("Id", ""):
            return i
    raise RuntimeError(f"camera {id_substr} introuvable")


class Cam:
    def __init__(self, node, c):
        self.node = node; self.c = c; self.rot = c["rot180"]
        self.pub = node.create_publisher(
            CompressedImage, f"/head_camera/{c['side']}/image_raw/compressed", qos_profile_sensor_data)
        self.enc = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        self.cam = Picamera2(pick(c["cam"]))
        self.cam.configure(self.cam.create_still_configuration(main={"size": (640, 480), "format": "RGB888"}))
        self.cam.start()
        fd = int(1e6 / 15)
        self.locked = {"AeEnable": False, "AwbEnable": False, "ExposureTime": c["exposure"],
                       "AnalogueGain": c["gain"], "ColourGains": (c["red"], c["blue"]),
                       "FrameDurationLimits": (fd, fd)}
        self.cam.set_controls(self.locked); time.sleep(1.0)
        self.running = True
        self.thr = threading.Thread(target=self._loop, daemon=True); self.thr.start()

    def _loop(self):
        k = 0; n = 0; t0 = time.monotonic()
        while self.running and rclpy.ok():
            ts = time.monotonic(); k += 1
            if k % 15 == 0:
                self.cam.set_controls(self.locked)   # re-assert vs reset concurrence
            f = self.cam.capture_array("main")
            if self.rot:
                f = cv2.rotate(f, cv2.ROTATE_180)
            ok, j = cv2.imencode(".jpg", f, self.enc)
            if ok:
                m = CompressedImage()
                m.header.stamp = self.node.get_clock().now().to_msg()
                m.header.frame_id = f"head_camera_{self.c['side']}"
                m.format = "jpeg"; m.data = j.tobytes()
                self.pub.publish(m); n += 1
            if time.monotonic() - t0 >= 5:
                self.node.get_logger().info(f"[{self.c['side']}] {n/(time.monotonic()-t0):.1f} fps FIGE")
                n = 0; t0 = time.monotonic()
            sl = 1.0 / 15 - (time.monotonic() - ts)
            if sl > 0:
                time.sleep(sl)


def main():
    rclpy.init()
    node = rclpy.create_node("cam_pub_dual")
    cams = [Cam(node, c) for c in CAMS]   # LES DEUX dans un seul process / CameraManager
    node.get_logger().info(f"{len(cams)} cameras FIGEES lancees")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for c in cams:
            c.running = False
        for c in cams:
            try:
                c.cam.stop(); c.cam.close()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
