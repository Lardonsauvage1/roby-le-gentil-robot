#!/usr/bin/env python3
"""Serveur MJPEG mono-camera commutable pour Pi5 (shim libcamera).
Les 2 cameras (imx219 cam0 + ov5647 cam1) partagent l'ISP PiSP : ni l'acces
concurrent ni la REOUVERTURE in-process ne marchent (la 2e camera se bloque).
=> chaque bascule tue le sous-process de capture courant et en lance un NEUF
(cam_capture.py) pour la camera choisie. Une seule camera active a la fois.
Lancer : setsid env LD_PRELOAD=<v4l2-compat.so> python3 cam_switch.py
Vue : http://192.168.2.37:8080/
"""
import os, time, signal, subprocess, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = int(os.environ.get('CAM_PORT', '8080'))
FRAME = "/dev/shm/roby_cam.jpg"
SHIM = "/usr/libexec/aarch64-linux-gnu/libcamera/v4l2-compat.so"
CAP = os.path.expanduser("~/cam_capture.py")
FPS = 15
# Mapping shim (verifie 2026-06-26) : idx0=imx219(cam0), idx8=ov5647(cam1).
# 2x OV5647 depuis 2026-06-26 (Pi Cam 1.3 neuve sur cam0 + ov5647 d'origine cam1)
DEV = {0: 0, 1: 8}
NAMES = {0: "CAM 0 (ov5647 cam0)", 1: "CAM 1 (ov5647 cam1)"}

state = {'cam': -1, 'proc': None, 'lock': threading.Lock()}


def start_cam(cam):
    with state['lock']:
        p = state['proc']
        if p is not None:
            try:
                # SIGTERM => le process fait cap.release() (libere l'ISP partage)
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                p.wait(timeout=6)
            except Exception:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
            time.sleep(1.5)  # laisser l'ISP PiSP se liberer avant la cam suivante
        state['proc'] = None
        try:
            os.remove(FRAME)  # eviter de montrer une frame figee de l'ancienne cam
        except FileNotFoundError:
            pass
        env = dict(os.environ)
        env['LD_PRELOAD'] = SHIM
        proc = subprocess.Popen(
            ['python3', CAP, str(DEV[cam]), NAMES[cam]],
            env=env, preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        state['proc'] = proc
        state['cam'] = cam


def read_frame():
    try:
        with open(FRAME, 'rb') as fh:
            return fh.read()
    except (FileNotFoundError, OSError):
        return None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        p = self.path
        if p.startswith('/switch'):
            try:
                cam = int(p.split('cam=')[1][0])
            except Exception:
                cam = 0
            if cam in DEV:
                start_cam(cam)
            self.send_response(302)
            self.send_header('Location', '/')
            self.end_headers()
            return
        if p == '/' or p.startswith('/index'):
            act = state['cam']

            def btn(c):
                on = ' style="background:#2a7;color:#fff"' if c == act else ''
                return (f'<a href="/switch?cam={c}">'
                        f'<button{on}>{NAMES[c]}</button></a>')

            html = (
                "<html><head><title>Cameras Roby</title>"
                "<style>body{background:#222;color:#eee;text-align:center;"
                "font-family:sans-serif}img{width:90vw;max-width:900px;border:2px "
                "solid #444;background:#000}button{font-size:18px;padding:10px 20px;"
                "margin:6px;cursor:pointer}</style></head><body>"
                f"<h3>Positionnement cameras &mdash; active: {NAMES.get(act,'...')}</h3>"
                f"{btn(0)}{btn(1)}"
                "<p style='color:#999;font-size:13px'>1 camera a la fois (ISP "
                "partage). Apres un clic, ~3-5s pour l'image (init + auto-expo).</p>"
                "<br><img src='/stream'></body></html>").encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(html)
            return
        if p == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    jpg = read_frame()
                    if jpg is None or len(jpg) < 100:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                    self.wfile.write(
                        f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                    time.sleep(1.0 / FPS)
            except (BrokenPipeError, ConnectionResetError):
                return
        self.send_error(404)


def main():
    start_cam(0)  # demarre sur CAM 0 (imx219)
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f"cam_switch sur http://0.0.0.0:{PORT}/", flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
