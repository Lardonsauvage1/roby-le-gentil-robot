#!/usr/bin/env python3
"""cam_pub_pi2_dual.py — Les 2 caméras picamera2 dans UN SEUL process (CameraManager
partagé). Necessaire : 2 process separes cassent le verrouillage manuel (concurrence ISP).
Contrôles exposition/gain/WB FIGES + re-assertes => rendu reproductible."""
import threading
import os
import time

import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
# LE DEFAUT EST *BRUT* (decision Sam, 2026-09-06) : aucune correction couleur ni
# luminosite ajoutee. Les deux autres modes sont opt-in par variable d environnement.
#   (defaut)            -> BRUT      : rien. Cf. le detail des controles dans Cam.__init__.
#   ROBY_CAM_AUTO=1     -> TOUT-AUTO : exposition ET balance des blancs automatiques.
#   ROBY_CAM_FIGE=1     -> TOUT-FIGE : expo/gain/WB fixes des CAMS ci-dessous.
_AUTO = os.environ.get("ROBY_CAM_AUTO", "") not in ("", "0")
_FIGE = os.environ.get("ROBY_CAM_FIGE", "") not in ("", "0")
# ROBY_CAM_BRUT=1 reste accepte pour l ecrire explicitement, mais ne change rien au defaut.
_BRUT = (os.environ.get("ROBY_CAM_BRUT", "") not in ("", "0")) or not (_AUTO or _FIGE)

# Table de calibration NoIR. Ces modules n ont pas de filtre infrarouge : avec la table
# standard ov5647.json l IR deborde sur le rouge et le bleu (mesure 2026-09-06 :
# R/V=1.405 B/V=1.316 en standard contre 0.979/1.034 en noir). MAIS cette table est
# elle-meme une correction couleur (matrice CCM, correction d objectif, gamma) : en BRUT
# on ne l impose donc pas, et libcamera reprend ov5647.json. Derive IR visible = ASSUME.
# A poser AVANT tout usage de libcamera : le CameraManager est un singleton, et passer
# tuning= a Picamera2 apres un global_camera_info() est sans effet.
_TUN = "/usr/share/libcamera/ipa/rpi/pisp/ov5647_noir.json"
if not _BRUT and os.path.exists(_TUN):
    os.environ.setdefault("LIBCAMERA_RPI_TUNING_FILE", _TUN)

from picamera2 import Picamera2

# Valeurs FIGEES par cote (a re-tuner au besoin). left=EXTERIEURE i2c@88000, right=POIGNET i2c@80000.
CAMS = [
    dict(side="left",  cam="i2c@88000", rot180=False,  exposure=66640, gain=6.875, red=1.039, blue=1.616),
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
        # Table de calibration : ces modules n'ont PAS de filtre infrarouge (NoIR). Avec la
        # table standard ov5647.json, l'IR deborde sur le rouge et le bleu => dominante rose
        # que meme la balance des blancs AUTO ne rattrape pas (mesure 2026-09-06 :
        # R/V=1.405 B/V=1.316 en standard, 0.979/1.034 en noir). Constat verifie sur la scene.
        self.cam = Picamera2(idx)
        self.cam.configure(self.cam.create_still_configuration(main={"size": (640, 480), "format": "RGB888"}))
        self.cam.start()
        fd = int(1e6 / 15)
        # MODE BRUT = LE DEFAUT. On retire tout ce que l ISP ajoute sur les couleurs
        # et la luminosite, dans la limite de ce que libcamera expose.
        #   - matrice de correction couleur forcee a l IDENTITE (sinon l ISP applique
        #     celle de la table de calibration : c est LE gros traitement couleur) ;
        #   - balance des blancs coupee ET gains a 1.0/1.0 : AwbEnable=False seul
        #     conserve les derniers gains calcules, ce n est pas neutre ;
        #   - saturation / contraste / luminosite aux valeurs neutres, accentuation a 0
        #     et debruitage OFF (ce sont bien des traitements, actifs par defaut) ;
        #   - exposition AUTO : la luminosite n est plus imposee par nous.
        # Restent INEVITABLES (pas de controle libcamera, c est cable dans le pipeline
        # PiSP) : niveau de noir, dematricage Bayer, correction d objectif et gamma.
        # Les enlever imposerait de publier le flux RAW Bayer et de dematricer nous-memes.
        self.brut = _BRUT
        self.auto = _AUTO
        self.fige = _FIGE
        if self.brut:
            self.locked = {"AeEnable": True,
                           "AwbEnable": False, "ColourGains": (1.0, 1.0),
                           "ColourCorrectionMatrix": (1.0, 0.0, 0.0,
                                                      0.0, 1.0, 0.0,
                                                      0.0, 0.0, 1.0),
                           "Saturation": 1.0, "Contrast": 1.0, "Brightness": 0.0,
                           "Sharpness": 0.0, "NoiseReductionMode": 0,
                           "FrameDurationLimits": (fd, fd)}
            self.mode = "BRUT"
        elif self.auto:
            self.locked = {"AeEnable": True, "AwbEnable": True,
                           "FrameDurationLimits": (fd, fd)}
            self.mode = "TOUT-AUTO"
        elif self.fige:
            self.locked = {"AeEnable": False, "AwbEnable": False, "ExposureTime": c["exposure"],
                           "AnalogueGain": c["gain"], "ColourGains": (c["red"], c["blue"]),
                           "FrameDurationLimits": (fd, fd)}
            self.mode = "TOUT-FIGE"
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
                self.node.get_logger().info(
                    f"[{self.c['side']}] {n/(time.monotonic()-t0):.1f} fps {self.mode}")
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
        f"{len(cams)}/{len(CAMS)} cameras lancees en mode {cams[0].mode} : "
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
