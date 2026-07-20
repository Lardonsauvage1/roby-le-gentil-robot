#!/usr/bin/env python3
"""roby_cam_view.py — visualisation LIVE des 2 cameras dans un navigateur.

S'abonne aux topics ROS `/head_camera/{left,right}/image_raw/compressed` et les
rediffuse en MJPEG sur http://localhost:8081/ . Les images sont DEJA en JPEG dans
le topic : on relaie les octets tels quels (aucun decodage/reencodage).

⚠️ Ne touche PAS au materiel : c'est un simple consommateur de topics, donc AUCUN
conflit avec `cam_pub_pi2_dual.py` sur le Pi5 (contrairement a l'ancien cam_switch.py
qui ouvrait les cameras en direct et ne supportait pas de concurrent).

Usage : bash ~/roby_cam_view.sh     puis ouvrir http://localhost:8081/
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

PORT = 8081
CAMS = ("left", "right")

_frames = {c: None for c in CAMS}       # derniers octets JPEG
_stamps = {c: 0.0 for c in CAMS}
_lock = threading.Lock()


class Bridge(Node):
    def __init__(self):
        super().__init__("roby_cam_view")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        for c in CAMS:
            self.create_subscription(
                CompressedImage, f"/head_camera/{c}/image_raw/compressed",
                self._mk(c), qos)

    def _mk(self, cam):
        def cb(msg):
            with _lock:
                _frames[cam] = bytes(msg.data)
                _stamps[cam] = time.time()
        return cb


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>Roby — cameras live</title>
<style>
 body{background:#141414;color:#ddd;font-family:system-ui,sans-serif;margin:0;padding:16px}
 h1{font-size:16px;font-weight:600;margin:0 0 12px}
 .row{display:flex;gap:16px;flex-wrap:wrap}
 .cam{background:#1e1e1e;border-radius:8px;padding:10px;flex:1 1 480px}
 .cam h2{font-size:13px;margin:0 0 8px;color:#9ad}
 img{width:100%;height:auto;border-radius:4px;display:block;background:#000}
 .note{color:#777;font-size:12px;margin-top:12px}
</style></head><body>
<h1>Roby — cameras live (15 Hz)</h1>
<div class="row">
  <div class="cam"><h2>LEFT — exterieure (obs du reseau)</h2><img src="/stream/left"></div>
  <div class="cam"><h2>RIGHT — poignet</h2><img src="/stream/right"></div>
</div>
<p class="note">Relais MJPEG des topics ROS. Aucun acces direct au materiel :
ce visualiseur n'entre pas en conflit avec le noeud camera du Pi5.</p>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                                  # pas de spam console

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/stream/"):
            cam = self.path.rsplit("/", 1)[-1]
            if cam not in CAMS:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last = 0.0
            try:
                while True:
                    with _lock:
                        buf, ts = _frames[cam], _stamps[cam]
                    if buf is not None and ts != last:
                        last = ts
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(buf)).encode()
                                         + b"\r\n\r\n" + buf + b"\r\n")
                    else:
                        time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError):
                return                        # onglet ferme : normal
        self.send_error(404)


def main():
    rclpy.init()
    node = Bridge()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    srv.daemon_threads = True
    print(f"visualiseur pret : http://localhost:{PORT}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
