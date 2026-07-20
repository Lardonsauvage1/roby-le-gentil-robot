#!/usr/bin/env python3
"""roby_collect_panel.py — Panneau de collecte dataset 1-par-1 avec tri manuel.

Boucle : enregistre 1 episode (le bras BOUGE) -> Sam clique GARDER ou JETER ->
GARDER range l'episode dans le dossier de session (renumerote) et RELANCE
automatiquement le suivant ; JETER supprime et relance ; STOP arrete la boucle.

Chaque run de l'oracle cree son propre batch dans une zone de TRANSIT (.staging) ;
seuls les episodes GARDES sont deplaces dans le batch de session consolide.

Env : lance via roby_collect_panel.sh (python systeme 3.12 + ROS + DDS domaine 42).
"""
import datetime
import glob
import json
import os
import queue
import shutil
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

HOME = os.path.expanduser("~")
DATASETS = os.path.join(HOME, "roby_datasets")
STAGING = os.path.join(DATASETS, ".staging")
ORACLE_SH = os.path.join(HOME, "roby_oracle_real.sh")
VIDEO_SRV = os.path.join(HOME, "bag_video_server.py")
VIDEO_PORT = 8091


class Panel:
    def __init__(self, root):
        self.root = root
        root.title("Roby — collecte dataset (tri 1-par-1)")
        self.q = queue.Queue()
        self.state = "idle"          # idle | recording | review | stopping
        self.stopping = False
        self.staging_batch = None    # dossier batch de transit de l'episode courant
        self.discarded = 0           # jetes DANS cette session de panneau
        self.video_proc = None
        self.worker = None

        os.makedirs(STAGING, exist_ok=True)
        # Session consolidee : REPREND la plus recente batch_collect_* si elle existe
        # (ou celle pointee par ROBY_COLLECT_DIR), sinon en cree une neuve. => on peut
        # rallumer le robot et continuer le MEME dataset. Numerotation = max(ep) + 1
        # (robuste aux trous : un ep supprime ne provoque pas de collision).
        self._pick_session(resume=True)

        self._build_ui()
        self._refresh_session_label()
        self.root.after(120, self._poll)

    def _pick_session(self, resume=True):
        forced = os.environ.get("ROBY_COLLECT_DIR", "").strip()
        if forced:
            self.session_dir = os.path.expanduser(forced)
        elif resume:
            cands = sorted(glob.glob(os.path.join(DATASETS, "batch_collect_*")),
                           key=os.path.getmtime)
            if cands:
                self.session_dir = cands[-1]
            else:
                self._new_session_dir()
        else:
            self._new_session_dir()
        os.makedirs(self.session_dir, exist_ok=True)
        self._scan_session()

    def _new_session_dir(self):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(DATASETS, f"batch_collect_{ts}")

    def _scan_session(self):
        """kept = nb d'ep presents ; next_idx = max index + 1 (robuste aux trous)."""
        eps = [d for d in glob.glob(os.path.join(self.session_dir, "ep_*"))
               if os.path.isdir(d) and os.path.basename(d)[3:].isdigit()]
        idxs = [int(os.path.basename(d)[3:]) for d in eps]
        self.kept = len(idxs)
        self.next_idx = (max(idxs) + 1) if idxs else 0

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        pad = dict(padx=8, pady=4)
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="both", expand=True)

        ttk.Label(top, text="Session :").grid(row=0, column=0, sticky="w", **pad)
        self.session_var = tk.StringVar(value="")
        ttk.Label(top, textvariable=self.session_var, foreground="#555",
                  wraplength=430, justify="left").grid(row=0, column=1, columnspan=2, sticky="w", **pad)
        self.b_newsess = ttk.Button(top, text="Nouvelle session", command=self.on_new_session)
        self.b_newsess.grid(row=0, column=3, sticky="e", **pad)

        # Vitesses editables (appliquees a l'episode suivant)
        ttk.Label(top, text="Vitesse libre (--vel) :").grid(row=1, column=0, sticky="w", **pad)
        self.vel = tk.StringVar(value="0.5")
        ttk.Entry(top, textvariable=self.vel, width=7).grid(row=1, column=1, sticky="w", **pad)
        ttk.Label(top, text="Ligne droite (--cart-speed m/s) :").grid(row=1, column=2, sticky="w", **pad)
        self.cart = tk.StringVar(value="0.04")
        ttk.Entry(top, textvariable=self.cart, width=7).grid(row=1, column=3, sticky="w", **pad)

        # Statut
        self.status = tk.StringVar(value="Prêt. Clique DÉMARRER (le bras va bouger).")
        self.status_lbl = tk.Label(top, textvariable=self.status, font=("TkDefaultFont", 13, "bold"),
                                   fg="#0a58ca", wraplength=560, justify="left")
        self.status_lbl.grid(row=2, column=0, columnspan=4, sticky="w", **pad)

        self.counts = tk.StringVar(value="Gardés : 0    Jetés : 0")
        ttk.Label(top, textvariable=self.counts, font=("TkDefaultFont", 11)).grid(
            row=3, column=0, columnspan=4, sticky="w", **pad)

        # Boutons decision
        btns = ttk.Frame(top)
        btns.grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 4))
        self.b_start = tk.Button(btns, text="▶ DÉMARRER", bg="#0a58ca", fg="white",
                                 font=("TkDefaultFont", 12, "bold"), width=13, command=self.on_start)
        self.b_start.grid(row=0, column=0, padx=4)
        self.b_keep = tk.Button(btns, text="✓ GARDER", bg="#198754", fg="white",
                                font=("TkDefaultFont", 12, "bold"), width=12, command=self.on_keep,
                                state="disabled")
        self.b_keep.grid(row=0, column=1, padx=4)
        self.b_drop = tk.Button(btns, text="✗ JETER", bg="#dc3545", fg="white",
                                font=("TkDefaultFont", 12, "bold"), width=12, command=self.on_drop,
                                state="disabled")
        self.b_drop.grid(row=0, column=2, padx=4)
        self.b_video = tk.Button(btns, text="▶ Voir vidéo", width=12, command=self.on_video,
                                 state="disabled")
        self.b_video.grid(row=0, column=3, padx=4)
        self.b_stop = tk.Button(btns, text="⏹ STOP", bg="#6c757d", fg="white",
                                font=("TkDefaultFont", 12, "bold"), width=10, command=self.on_stop)
        self.b_stop.grid(row=0, column=4, padx=4)

        # Log
        ttk.Label(top, text="Journal :").grid(row=5, column=0, sticky="w", **pad)
        self.log = tk.Text(top, height=12, width=78, font=("TkFixedFont", 9))
        self.log.grid(row=6, column=0, columnspan=5, sticky="nsew", padx=8)
        self.log.configure(state="disabled")

    def _logln(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_buttons(self, start, keep, drop, video):
        self.b_start.configure(state="normal" if start else "disabled")
        self.b_keep.configure(state="normal" if keep else "disabled")
        self.b_drop.configure(state="normal" if drop else "disabled")
        self.b_video.configure(state="normal" if video else "disabled")

    # ------------------------------------------------------------- actions
    def on_start(self):
        if self.state != "idle":
            return
        self.stopping = False
        self._start_episode()

    def _start_episode(self):
        self.state = "recording"
        n = self.next_idx
        self.status.set(f"⏳ ENREGISTREMENT ep_{n:03d} — LE BRAS BOUGE (doigt sur la coupure)…")
        self.status_lbl.configure(fg="#b8860b")
        self._set_buttons(False, False, False, False)
        try:
            vel = float(self.vel.get()); cart = float(self.cart.get())
        except ValueError:
            self.status.set("❌ Vitesses invalides — corrige les champs.")
            self.state = "idle"; self._set_buttons(True, False, False, False)
            return
        self._logln(f"--- ep_{n:03d} : oracle --vel {vel} --cart-speed {cart} ---")
        self.worker = threading.Thread(target=self._run_oracle, args=(vel, cart), daemon=True)
        self.worker.start()

    def _run_oracle(self, vel, cart):
        """Thread : lance l'oracle 1 episode vers la zone de transit, remonte le resultat."""
        env = dict(os.environ)
        env["ROBY_CAMS"] = "both"
        cmd = ["bash", ORACLE_SH, "--episodes", "1", "--out", STAGING,
               "--vel", str(vel), "--cart-speed", str(cart)]
        batch_path = None
        ok = False
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, env=env)
            for line in p.stdout:
                line = line.rstrip("\n")
                if "Bags ->" in line:
                    batch_path = line.split("Bags ->", 1)[1].strip()
                if "OK (objet revenu" in line:
                    ok = True
                # remonte quelques lignes utiles au journal
                if any(k in line for k in ("Episode", "pince", "ECHEC", "seed", "Bags ->", "OK (")):
                    self.q.put(("log", line.replace("[INFO] ", "").strip()))
            p.wait()
        except Exception as e:  # noqa
            self.q.put(("log", f"[panel] erreur oracle : {e}"))
        # fallback : plus recent batch dans .staging si pas parse
        if batch_path is None:
            cands = sorted(glob.glob(os.path.join(STAGING, "batch_*")), key=os.path.getmtime)
            batch_path = cands[-1] if cands else None
        self.q.put(("done", {"batch": batch_path, "ok": ok}))

    @staticmethod
    def _bag_a_des_images(ep_dir):
        """(ok, detail) — le bag contient-il vraiment des images des 2 cameras ?

        Lit le metadata.yaml ecrit par rosbag2 (pas besoin de `ros2 bag info`, qui est
        lent et depend de l'environnement ROS). On compte les messages par topic :
        un topic camera declare mais a 0 message = noeud tombe pendant l'episode."""
        meta = os.path.join(ep_dir, "metadata.yaml")
        if not os.path.isfile(meta):
            return False, "metadata.yaml absent"
        try:
            txt = open(meta, encoding="utf-8", errors="replace").read()
        except Exception as e:
            return False, f"metadata illisible ({e})"
        counts = {}
        topic = None
        for line in txt.splitlines():
            st = line.strip()
            if st.startswith("- topic_metadata:") or st.startswith("topic_metadata:"):
                topic = None
            if st.startswith("name:") and "/head_camera/" in st:
                topic = st.split("name:", 1)[1].strip()
            elif st.startswith("message_count:") and topic:
                try:
                    counts[topic] = int(st.split(":", 1)[1].strip())
                except ValueError:
                    pass
                topic = None
        if not counts:
            return False, "aucun topic camera dans le bag"
        vides = [t for t, c in counts.items() if c == 0]
        if vides:
            return False, "0 image sur " + ", ".join(os.path.basename(v) for v in vides)
        return True, " / ".join(f"{t.split('/')[2]}={c}" for t, c in sorted(counts.items()))

    def _on_done(self, info):
        batch = info["batch"]; ok = info["ok"]
        self.staging_batch = batch
        ep_dir = os.path.join(batch, "ep_000") if batch else None
        # FIX 2026-07-20 : on validait la seule PRESENCE d'un .mcap. Un bag sans
        # aucune image (noeud camera tombe en cours d'episode) passait donc le test et
        # etait propose en GARDER -- c'est precisement le piege vecu le 2026-07-15
        # (bag de 1.7 Mo, 0 image, decouvert bien plus tard). On lit desormais le
        # metadata.yaml du bag et on exige des messages sur les topics camera.
        has_bag = ep_dir and os.path.isdir(ep_dir) and glob.glob(os.path.join(ep_dir, "*.mcap"))
        img_ok, img_detail = self._bag_a_des_images(ep_dir) if has_bag else (False, "pas de bag")
        if has_bag and not img_ok:
            self._logln(f"  ❌ bag SANS IMAGES ({img_detail}) → jeté : inutilisable pour l'apprentissage.")
        if not ok or not has_bag or not img_ok:
            # echec oracle ou bag vide : auto-jete + relance (sauf stop)
            self.discarded += 1
            self._refresh_counts()
            self._logln(f"  ❌ épisode raté/incomplet (ok={ok}) → jeté automatiquement.")
            self._cleanup_staging()
            if self.stopping:
                self._go_idle("Arrêté.")
            else:
                self.status.set("↻ Épisode raté — relance automatique…")
                self.root.after(800, self._start_episode)
            return
        # episode pret : attente decision
        self.state = "review"
        self.status.set(f"👀 ep_{self.next_idx:03d} prêt — GARDER ou JETER ? (▶ Voir vidéo pour revoir)")
        self.status_lbl.configure(fg="#0a58ca")
        self._set_buttons(False, True, True, True)

    def on_keep(self):
        if self.state != "review":
            return
        n = self.next_idx
        dst = os.path.join(self.session_dir, f"ep_{n:03d}")
        try:
            shutil.move(os.path.join(self.staging_batch, "ep_000"), dst)
            # meta.json + rec.log (renumerotes)
            self._move_side(f"ep_000.meta.json", f"ep_{n:03d}.meta.json", set_ep=n)
            self._move_side(f"ep_000.rec.log", f"ep_{n:03d}.rec.log")
        except Exception as e:  # noqa
            # FIX 2026-07-20 : avant, next_idx et kept etaient incrementes MEME en cas
            # d'echec du rangement. L'episode etait donc perdu, le compteur mentait, et
            # un trou silencieux apparaissait dans la numerotation. On sort sans compter.
            self._logln(f"  ⚠️ ÉCHEC du rangement : {e}")
            self._logln(f"     épisode NON compté (numérotation inchangée : ep_{n:03d} reste libre)")
            self.discarded += 1
            self._refresh_counts()
            self._cleanup_staging()
            if self.stopping:
                self._go_idle("Arrêté.")
            else:
                self.status.set("↻ Rangement échoué — relance automatique…")
                self.root.after(800, self._start_episode)
            return
        self.next_idx += 1
        self.kept += 1
        self._refresh_counts()
        self._refresh_session_label()
        self._logln(f"  ✓ GARDÉ → {os.path.basename(dst)}")
        self._cleanup_staging()
        self._next_or_stop()

    def on_drop(self):
        if self.state != "review":
            return
        self.discarded += 1
        self._refresh_counts()
        self._logln("  ✗ JETÉ.")
        self._cleanup_staging()
        self._next_or_stop()

    def _move_side(self, src_name, dst_name, set_ep=None):
        src = os.path.join(self.staging_batch, src_name)
        if not os.path.exists(src):
            return
        dst = os.path.join(self.session_dir, dst_name)
        shutil.move(src, dst)
        if set_ep is not None and dst.endswith(".json"):
            try:
                with open(dst) as f:
                    d = json.load(f)
                d["episode"] = set_ep
                with open(dst, "w") as f:
                    json.dump(d, f, indent=2, ensure_ascii=False)
            except Exception:  # noqa
                pass

    def _next_or_stop(self):
        self._stop_video()
        self._set_buttons(False, False, False, False)
        if self.stopping:
            self._go_idle(f"Arrêté. Session : {self.kept} gardés dans {os.path.basename(self.session_dir)}")
        else:
            self.status.set("↻ Relance automatique de l'épisode suivant…")
            self.root.after(600, self._start_episode)

    def on_stop(self):
        self.stopping = True
        if self.state == "recording":
            self.status.set("⏹ Arrêt demandé — fin de l'épisode en cours puis stop (décision à prendre dessus).")
        elif self.state == "review":
            self.status.set("⏹ Arrêt demandé — prends la décision (GARDER/JETER) sur cet épisode, puis stop.")
        else:
            self._go_idle("Arrêté.")

    def _go_idle(self, msg):
        self.state = "idle"
        self.stopping = False
        self.status.set(msg)
        self.status_lbl.configure(fg="#198754")
        self._set_buttons(True, False, False, False)

    # ------------------------------------------------------------- video
    def on_video(self):
        if not self.staging_batch:
            return
        ep = os.path.join(self.staging_batch, "ep_000")
        self._stop_video()
        try:
            self.video_proc = subprocess.Popen(
                ["python3", VIDEO_SRV, ep, str(VIDEO_PORT)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.root.after(1200, lambda: subprocess.Popen(
                ["xdg-open", f"http://localhost:{VIDEO_PORT}/"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            self._logln(f"  ▶ vidéo : http://localhost:{VIDEO_PORT}/")
        except Exception as e:  # noqa
            self._logln(f"  ⚠️ vidéo indispo : {e}")

    def _stop_video(self):
        if self.video_proc and self.video_proc.poll() is None:
            try:
                self.video_proc.send_signal(signal.SIGINT)
                self.video_proc.wait(timeout=2)
            except Exception:  # noqa
                try:
                    self.video_proc.kill()
                except Exception:  # noqa
                    pass
        self.video_proc = None

    # ------------------------------------------------------------- misc
    def _cleanup_staging(self):
        if self.staging_batch and os.path.isdir(self.staging_batch):
            shutil.rmtree(self.staging_batch, ignore_errors=True)
        self.staging_batch = None

    def _refresh_counts(self):
        self.counts.set(f"Gardés : {self.kept}    Jetés : {self.discarded}")

    def _refresh_session_label(self):
        base = os.path.basename(self.session_dir)
        tag = "REPRISE" if self.kept else "neuve"
        self.session_var.set(f"{base}\n({tag} — {self.kept} épisode(s), prochain = ep_{self.next_idx:03d})")

    def on_new_session(self):
        if self.state not in ("idle",):
            self.status.set("⏹ Termine/arrête la boucle avant de créer une nouvelle session.")
            return
        self._new_session_dir()
        os.makedirs(self.session_dir, exist_ok=True)
        self.kept = 0; self.next_idx = 0; self.discarded = 0
        self._refresh_counts(); self._refresh_session_label()
        self._logln(f"[panel] nouvelle session : {os.path.basename(self.session_dir)}")

    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "log":
                    self._logln("    " + payload)
                elif kind == "done":
                    self._on_done(payload)
        except queue.Empty:
            pass
        self.root.after(120, self._poll)


def main():
    root = tk.Tk()
    app = Panel(root)

    def on_close():
        app._stop_video()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
