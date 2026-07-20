#!/usr/bin/env python3
"""Rejoue les 2 cameras d'un bag en MJPEG dans le navigateur (Firefox lit ca
nativement, aucun codec/lecteur a installer). Cote a cote, en boucle, vraie vitesse.
Usage : python3 bag_video_server.py <bag> [port]  -> http://localhost:<port>/
"""
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

BAG = sys.argv[1]
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8091

frames = {"left": [], "right": []}   # (t_rel, jpeg_bytes)


def load():
    r = SequentialReader()
    r.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("", ""))
    t0 = None
    while r.has_next():
        tp, d, t = r.read_next()
        for s in ("left", "right"):
            if tp == "/head_camera/%s/image_raw/compressed" % s:
                m = deserialize_message(d, CompressedImage)
                if t0 is None:
                    t0 = t
                frames[s].append(((t - t0) / 1e9, bytes(m.data)))


load()
print("frames: left=%d right=%d" % (len(frames["left"]), len(frames["right"])))

HTML = (
    "<html><body style='margin:0;background:#111;color:#ddd;font-family:sans-serif'>"
    "<div style='text-align:center;padding:6px'>recording ep_000 &mdash; camera GAUCHE / DROITE (boucle, vitesse reelle)</div>"
    "<div style='display:flex;justify-content:center;gap:6px'>"
    "<img src='/stream?cam=left' style='width:49vw'>"
    "<img src='/stream?cam=right' style='width:49vw'></div></body></html>"
)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML.encode())
            return
        if u.path == "/stream":
            cam = parse_qs(u.query).get("cam", ["left"])[0]
            fr = frames.get(cam, [])
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    t0 = time.monotonic()
                    for (trel, jpg) in fr:
                        while time.monotonic() - t0 < trel:
                            time.sleep(0.002)
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.4)   # petite pause avant de reboucler
            except Exception:
                return


print("Ouvre http://localhost:%d/  (Ctrl-C pour arreter)" % PORT)
ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
