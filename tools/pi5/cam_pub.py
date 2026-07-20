#!/usr/bin/env python3
"""
cam_pub.py — Publisher camera Pi5 pour la collecte de dataset (imitation learning).

- UNE camera par process (contrainte ISP PiSP partage : pas de 2 cams dans 1 process).
- Capture via OpenCV + shim libcamera v4l2-compat : le lanceur DOIT poser
  LD_PRELOAD=/usr/libexec/aarch64-linux-gnu/libcamera/v4l2-compat.so
  et forcer le backend V4L2 (cv2.CAP_V4L2), sinon OpenCV prend GStreamer et fige.
- Publie sensor_msgs/CompressedImage (JPEG) = le chemin de DEPLOIEMENT reel
  (le reseau Edimax 100 Mbit ne passe pas du raw). On enregistre a l'entree du
  reseau de neurones (sur le PC), donc on veut exactement ce flux compresse.
- Timestamp = now()-a-la-capture (horloge Pi5, la MEME que /joint_states) -> appariable.
  (le SensorTimestamp materiel viendra plus tard SI la mesure de latence le justifie.)

Usage (sur le Pi5, ROS source) :
  LD_PRELOAD=/usr/libexec/aarch64-linux-gnu/libcamera/v4l2-compat.so \
  python3 cam_pub.py --dev 0 --side left --width 640 --height 480 --quality 80
"""
import argparse
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CamPub(Node):
    def __init__(self, args):
        super().__init__(f"cam_pub_{args.side}")
        self.args = args
        topic = f"/head_camera/{args.side}/image_raw/compressed"
        # BEST_EFFORT (sensor_data) : QoS correcte pour un flux d'images haute cadence
        # (pas d'ACK a renvoyer -> insensible a un ufw qui bloque les retours, et on
        #  ne veut pas retransmettre des frames perimees). Le node reseau s'abonnera pareil.
        self.pub = self.create_publisher(CompressedImage, topic, qos_profile_sensor_data)

        self.cap = cv2.VideoCapture(args.dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if args.fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, args.fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"impossible d'ouvrir la camera dev={args.dev}")

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        reported_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"cam '{args.side}' dev={args.dev} -> {topic}  {w}x{h}  jpeg q{args.quality}  fps demande={args.fps} (V4L2 dit {reported_fps:.0f})"
        )
        self.enc_params = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
        self.frame_id = f"head_camera_{args.side}"
        # stats (fenetre glissante ~5 s)
        self._n = 0
        self._bytes = 0
        self._enc = 0.0
        self._t0 = time.monotonic()
        self._wb_lut = None  # LUT de balance des blancs (voir --wb-gains / --awb)
        if args.wb_gains:                                 # gains explicites B,G,R (calibres sur reference neutre)
            g = np.array([float(x) for x in args.wb_gains.split(",")], dtype=np.float32)
            self._build_wb_lut(g)
            self.get_logger().info(f"WB gains explicites : B={g[0]:.2f} G={g[1]:.2f} R={g[2]:.2f}")

    def _build_wb_lut(self, g):
        idx = np.arange(256, dtype=np.float32)
        self._wb_lut = np.clip(idx[None, :, None] * g[None, None, :], 0, 255).astype(np.uint8)

    def grab_and_publish(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("lecture frame KO")
            time.sleep(0.01)
            return
        if self.args.rot180:                              # camera montee a l'envers : ~0.2 ms, negligeable
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        if self.args.awb and self._wb_lut is None:        # grey-world auto : calibre sur 1ere frame ASSEZ LUMINEUSE
            s = frame[::4, ::4].reshape(-1, 3).mean(0)    # (l'expo met ~1s ; sinon frame noire -> gains 0)
            if s.mean() >= 20.0:
                g = s.mean() / np.clip(s, 1.0, None)
                self._build_wb_lut(g)
                self.get_logger().info(f"AWB grey-world fige : B={g[0]:.2f} G={g[1]:.2f} R={g[2]:.2f}")
        if self._wb_lut is not None:                      # applique la LUT (explicite ou grey-world) ~0.3 ms
            frame = cv2.LUT(frame, self._wb_lut)
        stamp = self.get_clock().now().to_msg()          # <-- now()-a-la-capture (horloge Pi5)
        t = time.monotonic()
        ok, buf = cv2.imencode(".jpg", frame, self.enc_params)
        enc = time.monotonic() - t
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.pub.publish(msg)

        self._n += 1
        self._bytes += len(msg.data)
        self._enc += enc
        dt = time.monotonic() - self._t0
        if dt >= 5.0:
            fps = self._n / dt
            kb = self._bytes / self._n / 1024.0
            mbps = self._bytes * 8 / dt / 1e6
            enc_ms = self._enc / self._n * 1000.0
            self.get_logger().info(
                f"[stats] {fps:5.1f} fps | {kb:5.0f} KB/frame | {mbps:5.1f} Mbit/s | encode {enc_ms:4.1f} ms"
            )
            self._n = 0
            self._bytes = 0
            self._enc = 0.0
            self._t0 = time.monotonic()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dev", type=int, default=0, help="index V4L2 (0=cam0 ov5647, 8=cam1)")
    p.add_argument("--side", default="left", help="left|right (nom du topic)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--quality", type=int, default=80, help="qualite JPEG 1-100")
    p.add_argument("--fps", type=int, default=0, help="fps demande (0=defaut camera)")
    p.add_argument("--rot180", action="store_true", help="rotation 180 (camera montee a l'envers)")
    p.add_argument("--awb", action="store_true", help="balance des blancs grey-world auto (scene-dependant)")
    p.add_argument("--wb-gains", default="", help="gains WB explicites 'B,G,R' (calibres sur ref neutre)")
    args = p.parse_args()

    rclpy.init()
    node = CamPub(args)
    try:
        while rclpy.ok():
            node.grab_and_publish()          # cap.read() cadence la boucle a la vitesse camera
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
