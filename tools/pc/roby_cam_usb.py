#!/usr/bin/env python3
"""roby_cam_usb.py — réglage + placement de la caméra USB OV4689 (« AK-Camera »).

Ouvre la caméra en MJPG 2688×1520 @30 fps (capteur ENTIER : le 1920×1080 de la
caméra est un recadrage qui perd du champ) et sert une page web :
  - vue LIVE (grille / croix pour la placer),
  - un réglage par contrôle V4L2 (exposition, balance des blancs, focus, …), appliqué
    à chaud et RELU sur la caméra,
  - mesures en continu : fps réels, luminance, % de pixels noirs/cramés, histogramme,
  - « Figer » : enregistre le préréglage dans ~/roby_cam_presets/<nom>.json,
  - « Capture » : image pleine résolution + réglages dans ~/roby_cam_captures/.

⚠️ La caméra revient à ses réglages d'usine quand on la débranche : le préréglage
~/roby_cam_presets/ov4689.json (s'il existe) est RÉappliqué au démarrage.
Constats (2026-09-23) : en exposition manuelle, le contrôle « gain » n'a AUCUN effet
(0→255 : luminance 87→89) ; la seule vraie source de lumière est le temps d'exposition.

Usage : bash ~/roby_cam_usb.sh      puis ouvrir http://localhost:8090/
"""
import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from roby_cam_usb_v4l2 import Controls, find_device, DEFAULT_NAME  # noqa: E402

PRESET_DIR = os.path.expanduser("~/roby_cam_presets")
CAPTURE_DIR = os.path.expanduser("~/roby_cam_captures")
DEFAULT_PRESET = os.path.join(PRESET_DIR, "ov4689.json")


class Camera:
    """Thread de capture : garde la dernière image + les mesures.

    Si la caméra décroche (déconnexion USB : vu le 2026-09-23, elle revient seule en
    ~1 s mais sur un AUTRE /dev/videoN et avec ses réglages d'USINE), on la rouvre.

    `cible` = les réglages VOULUS (préréglage chargé + changements faits sur la page).
    Toutes les 2 s, les contrôles relus sur la caméra sont comparés à la cible et remis
    s'ils ont dérivé. Vécu le 2026-09-23 : après un décrochage, l'exposition était
    retombée à 166 (usine) sans que rien ne le signale, puis ce 166 a été « figé ».
    """

    def __init__(self, dev, ctl, w, h, fps):
        self.dev, self.ctl, self.req = dev, ctl, (w, h, fps)
        self.lock = threading.Lock()
        self.frame, self.seq, self.t_last = None, 0, 0.0
        self.times = []
        self.stats = {}
        self.cible = {}                # réglages voulus : préréglage + changements de la page
        self.derives = []              # (heure, contrôle, lu, voulu) remis par la surveillance
        self.decrochages = []          # heures des reconnexions
        self._open()
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        w, h, fps = self.req
        self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.fmt = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), self.cap.get(cv2.CAP_PROP_FPS))

    def _reconnect(self):
        heure = datetime.now().strftime("%H:%M:%S")
        print(f"[{heure}] caméra muette → recherche…", flush=True)
        self.cap.release()
        while True:
            dev = find_device()
            if dev:
                try:
                    self.dev = dev
                    self.ctl.reopen(dev)
                    self._open()
                    ecarts = self.ctl.apply(self.cible) if self.cible else "aucune cible : caméra en réglages d'usine"
                    break
                except OSError:
                    pass
            time.sleep(1.0)
        with self.lock:
            self.decrochages.append(heure)
        print(f"[{datetime.now():%H:%M:%S}] reconnectée sur {dev}, réglages remis"
              + (f" — ÉCARTS {ecarts}" if ecarts else ""), flush=True)

    def _loop(self):
        t_stats = t_snap = 0.0
        t_ok = time.monotonic() + 10.0   # grâce à l'ouverture : 1re image pleine résolution lente (vu > 5 s)
        while True:
            ok, f = self.cap.read()
            now = time.monotonic()
            if not ok:
                if now - t_ok > 2.0:
                    self._reconnect()
                    t_ok = time.monotonic() + 10.0
                time.sleep(0.05)
                continue
            t_ok = now
            with self.lock:
                self.frame, self.seq, self.t_last = f, self.seq + 1, now
                self.times = [t for t in self.times if now - t < 2.0] + [now]
            if now - t_stats > 0.5:        # mesures 2×/s, sur une image réduite
                t_stats = now
                self._compute(f)
            if now - t_snap > 2.0:         # la caméra respecte-t-elle encore la cible ?
                t_snap = now
                self._surveiller()

    def _surveiller(self):
        try:
            lus = {c["key"]: c for c in self.ctl.list()}
        except OSError:
            return
        derive = {k: v for k, v in self.cible.items()
                  if k in lus and not lus[k]["inactive"] and lus[k]["value"] != v}
        if not derive:
            return
        heure = datetime.now().strftime("%H:%M:%S")
        for k, v in derive.items():
            print(f"[{heure}] DÉRIVE {k} : lu {lus[k]['value']}, voulu {v} → remis", flush=True)
            with self.lock:
                self.derives.append([heure, k, lus[k]["value"], v])
        try:
            self.ctl.apply(derive)
        except OSError:
            pass

    def _compute(self, f):
        small = cv2.resize(f, (480, 270), interpolation=cv2.INTER_AREA)
        g = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        hist = np.bincount(g.ravel() // 4, minlength=64)
        b, gr, r = (float(small[:, :, i].mean()) for i in range(3))
        with self.lock:
            self.stats = {"lum": round(float(g.mean()), 1),
                          "noir_pct": round(100 * float((g < 10).mean()), 1),
                          "blanc_pct": round(100 * float((g > 245).mean()), 1),
                          "bgr": [round(b), round(gr), round(r)],
                          "hist": (hist / hist.max()).round(3).tolist()}

    def state(self):
        with self.lock:
            n = len(self.times)
            fps = (n - 1) / (self.times[-1] - self.times[0]) if n > 2 else 0.0
            age = time.monotonic() - self.t_last if self.t_last else None
            return dict(self.stats, fps=round(fps, 1), seq=self.seq, dev=self.dev,
                        muette=age is None or age > 1.0, format=self.fmt,
                        decrochages=list(self.decrochages), derives=list(self.derives[-20:]))

    def latest(self):
        with self.lock:
            return self.frame, self.seq


def make_handler(cam, ctl, args):
    page_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roby_cam_usb.html")

    def controls_payload():
        return ctl.list()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                with open(page_path, "rb") as f:     # relue à chaque fois : modifiable à chaud
                    self.wfile.write(f.read())
            elif u.path == "/api/state":
                self._json({"stats": cam.state(), "controls": controls_payload(),
                            "dev": cam.state()["dev"], "presets": sorted(os.listdir(PRESET_DIR))
                            if os.path.isdir(PRESET_DIR) else []})
            elif u.path == "/stream.mjpg":
                self._stream(int(q.get("w", ["1280"])[0]))
            else:
                self.send_error(404)

        def _stream(self, width):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last = -1
            try:
                while True:
                    f, seq = cam.latest()
                    if f is None or seq == last:
                        time.sleep(0.01)
                        continue
                    last = seq
                    if width and width < f.shape[1]:
                        f = cv2.resize(f, (width, int(f.shape[0] * width / f.shape[1])),
                                       interpolation=cv2.INTER_AREA)
                    jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                if u.path == "/api/set":
                    got = ctl.set(q["key"], int(q["value"]))
                    cam.cible[q["key"]] = got      # un réglage fait sur la page devient voulu
                    self._json({"ok": True, "relu": got, "controls": controls_payload()})
                elif u.path == "/api/defaults":
                    cam.cible = {c["key"]: c["default"] for c in ctl.list()}
                    ecarts = ctl.apply(cam.cible)
                    self._json({"ok": True, "ecarts": ecarts, "controls": controls_payload()})
                elif u.path == "/api/preset/save":
                    self._json(save_preset(q.get("name") or "ov4689"))
                elif u.path == "/api/preset/load":
                    self._json(load_preset(os.path.join(PRESET_DIR, os.path.basename(q["file"]))))
                elif u.path == "/api/capture":
                    self._json(capture(q.get("label", "")))
                else:
                    self.send_error(404)
            except (OSError, KeyError, ValueError) as e:
                self._json({"ok": False, "erreur": str(e), "controls": controls_payload()}, 400)

    def save_preset(name):
        name = "".join(c for c in name if c.isalnum() or c in "-_") or "ov4689"
        os.makedirs(PRESET_DIR, exist_ok=True)
        path = os.path.join(PRESET_DIR, f"{name}.json")
        st = cam.state()
        controls = ctl.snapshot()
        if st["muette"] or not controls:
            return {"ok": False, "erreur": "caméra muette ou contrôles illisibles : rien enregistré"}
        lus = {c["key"]: c for c in ctl.list()}
        derive = {k: (lus[k]["value"], v) for k, v in cam.cible.items()
                  if k in lus and not lus[k]["inactive"] and lus[k]["value"] != v}
        if derive:
            return {"ok": False, "erreur": f"la caméra ne respecte pas les réglages voulus {derive} : rien enregistré"}
        doc = {"date": datetime.now().isoformat(timespec="seconds"), "camera": DEFAULT_NAME,
               "format": {"width": st["format"][0], "height": st["format"][1],
                          "fps": st["format"][2], "fourcc": "MJPG"},
               "mesures_a_l_enregistrement": {k: st.get(k) for k in
                                              ("fps", "lum", "noir_pct", "blanc_pct", "bgr")},
               "controls": controls}
        with open(path, "w") as f:
            json.dump(doc, f, indent=2, ensure_ascii=False)
        return {"ok": True, "fichier": path}

    def load_preset(path):
        with open(path) as f:
            controls = json.load(f)["controls"]
        cam.cible = dict(controls)
        ecarts = ctl.apply(controls)
        return {"ok": True, "fichier": path, "ecarts": ecarts, "controls": controls_payload()}

    def capture(label):
        f, _ = cam.latest()
        if f is None:
            return {"ok": False, "erreur": "aucune image"}
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        label = "".join(c for c in label if c.isalnum() or c in "-_")
        base = os.path.join(CAPTURE_DIR, datetime.now().strftime("%Y%m%d-%H%M%S")
                            + (f"_{label}" if label else ""))
        cv2.imwrite(base + ".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 95])
        st = cam.state()
        with open(base + ".json", "w") as fh:
            json.dump({"mesures": {k: st.get(k) for k in ("fps", "lum", "noir_pct", "blanc_pct", "bgr")},
                       "controls": ctl.snapshot()}, fh, indent=2)
        return {"ok": True, "fichier": base + ".jpg"}

    H.load_preset = staticmethod(load_preset)
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--width", type=int, default=2688)
    ap.add_argument("--height", type=int, default=1520)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--preset", default=DEFAULT_PRESET,
                    help="préréglage appliqué au démarrage s'il existe ('' = aucun)")
    ap.add_argument("--lan", action="store_true",
                    help="écouter sur le réseau local (ex. voir l'image sur un téléphone en la plaçant)")
    args = ap.parse_args()

    dev = find_device()
    if dev is None:
        print(f"Caméra « {DEFAULT_NAME} » introuvable : j'attends qu'elle apparaisse (`lsusb`)…", flush=True)
    while dev is None:       # rallonge limite : elle peut être décrochée au lancement
        time.sleep(1.0)
        dev = find_device()
    ctl = Controls(dev)
    cam = Camera(dev, ctl, args.width, args.height, args.fps)
    print(f"{dev} ouverte en {cam.fmt[0]}x{cam.fmt[1]} @{cam.fmt[2]:g} fps (demandé "
          f"{args.width}x{args.height} @{args.fps})")
    handler = make_handler(cam, ctl, args)
    if args.preset and os.path.exists(args.preset):
        r = handler.load_preset(args.preset)
        print(f"préréglage {args.preset} appliqué" + (f" — ÉCARTS {r['ecarts']}" if r["ecarts"] else ""))
    else:
        cam.cible = ctl.snapshot()
        print("aucun préréglage appliqué : l'état trouvé devient la cible surveillée")

    host = "0.0.0.0" if args.lan else "127.0.0.1"
    srv = ThreadingHTTPServer((host, args.port), handler)
    srv.daemon_threads = True
    print(f"→ http://localhost:{args.port}/   (Ctrl+C pour arrêter)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
