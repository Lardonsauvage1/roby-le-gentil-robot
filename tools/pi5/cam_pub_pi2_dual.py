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
    """Index de la camera dont l'Id contient id_substr, ou None si absente.

    Avant (2026-07-20) : levait une exception, donc UNE camera debranchee empechait
    le noeud de demarrer et privait de flux la camera SAINE. Or l'inference n'a
    besoin que de la gauche : une nappe debranchee cote poignet bloquait tout le
    deploiement pour rien. On saute desormais les absentes."""
    for i, info in enumerate(Picamera2.global_camera_info()):
        if id_substr in info.get("Id", ""):
            return i
    return None


class CameraAbsente(Exception):
    """Capteur non enumere (nappe debranchee, module HS). Non fatal : les autres
    cameras doivent continuer a publier."""
    pass


class Cam:
    def __init__(self, node, c):
        self.node = node; self.c = c; self.rot = c["rot180"]
        self.pub = node.create_publisher(
            CompressedImage, f"/head_camera/{c['side']}/image_raw/compressed", qos_profile_sensor_data)
        self.enc = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        idx = pick(c["cam"])
        if idx is None:
            raise CameraAbsente(c["cam"])
        self.cam = Picamera2(idx)
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
    # UN SEUL process pour toutes les cameras (CameraManager partage) : deux process
    # cassent le verrouillage expo/WB. Une camera absente est SAUTEE avec un
    # avertissement, les autres publient quand meme.
    cams = []
    for c in CAMS:
        try:
            cams.append(Cam(node, c))
        except CameraAbsente as e:
            node.get_logger().warn(
                f"camera '{c['side']}' ({e}) ABSENTE -> ignoree. "
                f"L'inference n'a besoin que de 'left' ; l'enregistrement d'episodes, lui, "
                f"exige les DEUX et sera inutilisable.")
    if not cams:
        node.get_logger().error("AUCUNE camera disponible -> arret.")
        rclpy.shutdown()
        return
    node.get_logger().info(
        f"{len(cams)}/{len(CAMS)} cameras FIGEES lancees : "
        f"{', '.join(c.c['side'] for c in cams)}")
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
