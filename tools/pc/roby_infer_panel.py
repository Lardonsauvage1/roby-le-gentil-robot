#!/usr/bin/env python3
"""Panneau minimal pour lancer/arrêter le modèle sur le vrai bras (2026-09-07).

Un seul geste utile : STOP. Il coupe l'inférence, laisse le garde figer la consigne,
puis ramène le bras à la pose de départ haute — celle sur laquelle le modèle a été
entraîné (médiane des débuts d'épisode du dataset) — donc prêt à relancer.

Le garde `roby_guard.py` est lancé/arrêté avec le panneau : l'inférence publie vers
/guard/joint_trajectory, rien n'atteint le bras sans lui.
"""
import os, signal, subprocess, threading, time
import tkinter as tk
from tkinter import ttk

# Seuils d'hysteresis de roby_gripper.py, recopies ici pour l'affichage seulement.
# Ils ne commandent rien : la decision reste prise dans le noeud d'inference.
GRIP_BAS, GRIP_HAUT = 0.4, 0.6

HOME = os.path.expanduser("~")
MODEL = os.environ.get("ROBY_INFER_MODEL", os.path.join(
    HOME, "lerobot-experiments/outputs/propre_263M_cooldown2/263M/pretrained_model"))
POSE_HAUTE = os.environ.get("ROBY_POSE_HAUTE", "depart_infer_comp")
PY_DEPLOY = os.path.join(HOME, "lerobot-experiments/venv/bin/python")

ENV = dict(os.environ)
ENV.setdefault("ROBY_J3_SCALE", "0.9299")
ENV.setdefault("ROBY_INFER_CAM", "right")
# Epinglage de l'inference sur les P-cores (Core Ultra 9 185H : CPU 0-11 = 6 P-cores
# + HT ; 12-19 E-cores, 20-21 LP-E). Sans lui, les 6 threads torch tombent en partie
# sur les E-cores et le plus lent retarde les autres. Mesure 2026-09-12, meme modele,
# bras immobile, DRY, sur CPU : 670 ms sans epinglage (hors budget 0,53 s) -> 497 ms
# avec. Utile aussi avec l'iGPU (decodage images, pre/post-traitement restent CPU).
# ROBY_INFER_CPUS="" desactive.
ENV.setdefault("OMP_NUM_THREADS", "6")
INFER_CPUS = os.environ.get("ROBY_INFER_CPUS", "0-11").strip()


class Panel:
    def __init__(self, root):
        self.root = root
        self.infer = None
        self.guard = None
        root.title("Roby — modèle sur le bras")
        root.geometry("560x460")

        ttk.Label(root, text=os.path.basename(os.path.dirname(os.path.dirname(MODEL))),
                  font=("TkDefaultFont", 11, "bold")).pack(pady=(12, 0))
        ttk.Label(root, text=MODEL.replace(HOME, "~"), foreground="#666").pack()

        self.etat = tk.StringVar(value="prêt — le bras ne bougera qu'au DÉMARRAGE")
        self.lbl = tk.Label(root, textvariable=self.etat, font=("TkDefaultFont", 11, "bold"),
                            fg="#0a58ca", wraplength=520)
        self.lbl.pack(pady=10)

        b = ttk.Frame(root); b.pack(pady=6)
        self.b_start = tk.Button(b, text="▶ DÉMARRER", bg="#198754", fg="white",
                                 font=("TkDefaultFont", 14, "bold"), width=14,
                                 command=self.on_start)
        self.b_start.grid(row=0, column=0, padx=8)
        self.b_stop = tk.Button(b, text="■ STOP\net remonter", bg="#dc3545", fg="white",
                                font=("TkDefaultFont", 16, "bold"), width=16, height=3,
                                command=self.on_stop, state="disabled")
        self.b_stop.grid(row=0, column=1, padx=8)

        # --- Pince : ce que le modele demande, en direct ----------------------
        # La DECISION seule ne suffit pas a comprendre : le modele sort une valeur
        # CONTINUE, et l'hysteresis ne ferme qu'au-dessus de 0,6 et n'ouvre qu'en
        # dessous de 0,4. Une valeur bloquee dans la bande donne donc une pince qui
        # ne bouge pas sans que rien ne soit en panne. On affiche les deux.
        pf = ttk.LabelFrame(root, text="Pince — consigne du modele")
        pf.pack(padx=10, pady=(4, 0), fill="x")
        self.grip_cmd = tk.StringVar(value="—")
        self.grip_lbl = tk.Label(pf, textvariable=self.grip_cmd,
                                 font=("TkDefaultFont", 13, "bold"), fg="#666")
        self.grip_lbl.pack(side="left", padx=10, pady=6)
        self.grip_bar = tk.Canvas(pf, width=320, height=26, highlightthickness=0)
        self.grip_bar.pack(side="right", padx=10, pady=6)
        self._draw_grip(None, None, False)

        self.log = tk.Text(root, height=9, width=68, font=("monospace", 9))
        self.log.pack(padx=10, pady=8, fill="both", expand=True)
        self._log("panneau prêt. Le garde sera lancé au démarrage.")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        threading.Thread(target=self._ecoute_pince, daemon=True).start()

    # ------------------------------------------------------- affichage pince
    def _draw_grip(self, valeur, ferme, frais):
        """Barre 0..1 avec la bande d'hysteresis et le curseur de la valeur."""
        c = self.grip_bar
        c.delete("all")
        W, H = 320, 26
        c.create_rectangle(0, 6, W, H - 6, fill="#eee", outline="#bbb")
        c.create_rectangle(GRIP_BAS * W, 6, GRIP_HAUT * W, H - 6,
                           fill="#ffe9a8", outline="")          # bande morte
        c.create_text(GRIP_BAS * W - 2, H // 2, text="ouvre", anchor="e",
                      font=("TkDefaultFont", 7), fill="#666")
        c.create_text(GRIP_HAUT * W + 2, H // 2, text="ferme", anchor="w",
                      font=("TkDefaultFont", 7), fill="#666")
        if valeur is not None:
            x = max(0.0, min(1.0, valeur)) * W
            col = "#dc3545" if ferme else "#198754"
            if not frais:
                col = "#999"                                    # consigne rejouee
            c.create_line(x, 2, x, H - 2, fill=col, width=3)

    def _maj_pince(self, valeur, ferme, frais):
        etat = "FERME" if ferme else "OUVRE"
        suffixe = "" if frais else "  (rejouee)"
        self.grip_cmd.set("%s   %.3f%s" % (etat, valeur, suffixe))
        self.grip_lbl.configure(fg="#dc3545" if ferme else "#198754")
        self._draw_grip(valeur, ferme, frais)

    def _ecoute_pince(self):
        """Ecoute les topics de pince du noeud d'inference, en lecture seule.

        Volontairement isole du chemin de commande : ce fil ne publie rien et ne
        peut donc pas influencer le bras. S'il echoue, le panneau reste utilisable
        et le dit une fois, plutot que de mourir en silence.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Bool, Float32
        except Exception as e:
            self.root.after(0, lambda: self._log(f"affichage pince indisponible : {e}"))
            return
        try:
            rclpy.init(args=None)
            n = Node("roby_infer_panel_pince")
            etat = {"ferme": False}

            def on_raw(m):
                v = float(m.data)
                frais = v >= 0.0                    # le noeud encode en negatif une
                if not frais:                       # consigne rejouee faute de neuf
                    v = -v - 1e-6
                self.root.after(0, self._maj_pince, v, etat["ferme"], frais)

            n.create_subscription(Float32, "/roby_infer/gripper_raw", on_raw, 10)
            n.create_subscription(Bool, "/guard/gripper",
                                  lambda m: etat.__setitem__("ferme", bool(m.data)), 10)
            rclpy.spin(n)
        except Exception as e:
            self.root.after(0, lambda: self._log(f"écoute pince arrêtée : {e}"))

    def _log(self, m):
        self.log.insert("end", f"[{time.strftime('%H:%M:%S')}] {m}\n")
        self.log.see("end")

    # ------------------------------------------------------------------ start
    def on_start(self):
        if self.infer:
            return
        self.b_start.configure(state="disabled")
        self.etat.set("démarrage du garde puis du modèle…")
        threading.Thread(target=self._start, daemon=True).start()

    def _start(self):
        if not self.guard or self.guard.poll() is not None:
            self.guard = subprocess.Popen(
                ["bash", os.path.join(HOME, "roby_guard.sh")],
                env=ENV, stdout=open("/tmp/guard.log", "w"),
                stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            self._log(f"garde lancé (PID {self.guard.pid})")
            time.sleep(4)
        cmd = [PY_DEPLOY, os.path.join(HOME, "roby_infer_cart.py"),
               "--model", MODEL, "--hz", "15", "--steps", "10",
               "--w-ori", "0.5", "--go"]
        if INFER_CPUS:
            cmd = ["taskset", "-c", INFER_CPUS] + cmd
        # iGPU PAR DEFAUT (decision du 2026-09-10, commit e39ccbd : "l'iGPU n'est pas une
        # option de confort"). L'ancien commentaire disait que le CPU tenait le budget
        # (0.368 s au banc) : faux en conditions reelles. Mesure 2026-09-12, modele du
        # 8 septembre, DRY : CPU 670 ms (497 ms epingle P-cores), iGPU 239 ms de mediane.
        # Le modele doit avoir son export OpenVINO (unet_ov.xml) a cote des poids.
        # ROBY_INFER_IGPU=non force le CPU (ex. : Arc occupe par un entrainement DAgger).
        igpu = os.environ.get("ROBY_INFER_IGPU", "GPU").strip()
        if igpu.lower() not in ("", "0", "non", "no", "none", "off"):
            cmd += ["--igpu", igpu]
        # --- Anti-saccade -----------------------------------------------------
        # Par defaut LeRobot execute les actions [1:9] de l'horizon = le futur
        # IMMEDIAT. Or l'inference prend ~368 ms, soit 5,5 pas a 15 Hz : ces actions
        # correspondent a une position DEJA DEPASSEE, le bras recule puis rattrape.
        #   ROBY_EXEC_OFFSET=6  -> decale la tranche pour compenser la latence
        #   ROBY_RTC=1          -> chunks recouvrants cousus par inpainting
        #                          (pas de pause NI de saut arriere)
        if os.environ.get("ROBY_RTC", "").strip() in ("1", "true", "oui"):
            cmd += ["--rtc"]
            for v, f in (("ROBY_RTC_DELAY", "--rtc-delay"), ("ROBY_RTC_GUIDE", "--rtc-guide")):
                if os.environ.get(v, "").strip():
                    cmd += [f, os.environ[v].strip()]
        elif os.environ.get("ROBY_EXEC_OFFSET", "").strip():
            cmd += ["--exec-offset", os.environ["ROBY_EXEC_OFFSET"].strip()]
        # Les sorties partaient dans /dev/null : un plantage du modele ou un refus du
        # garde etaient donc totalement muets, le panneau affichant "modele lance" sans
        # rien verifier. Vecu le 2026-09-10 : "pourquoi le modele ne fait rien ?" sans
        # aucune trace nulle part. On les ecrit desormais dans des fichiers relisibles.
        self.infer = subprocess.Popen(cmd, env=ENV, stdout=open("/tmp/infer.log", "w"),
                                      stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        self._log(f"modèle lancé (PID {self.infer.pid}) — le bras est AUTONOME")
        self.root.after(0, lambda: self.etat.set("⚠️ MODÈLE EN COURS — le bras bouge seul"))
        self.root.after(0, lambda: self.lbl.configure(fg="#dc3545"))
        self.root.after(0, lambda: self.b_stop.configure(state="normal"))

    # ------------------------------------------------------------------- stop
    def on_stop(self):
        self.b_stop.configure(state="disabled")
        self.etat.set("arrêt en cours…")
        self.lbl.configure(fg="#b8860b")
        threading.Thread(target=self._stop, daemon=True).start()

    def _stop(self):
        for nom, p in (("modèle", self.infer), ("garde", self.guard)):
            if p and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=5)
                except Exception:
                    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception: pass
                self._log(f"{nom} arrêté")
        self.infer = self.guard = None
        time.sleep(1.5)
        self._log(f"remontée vers « {POSE_HAUTE} »…")
        self.root.after(0, lambda: self.etat.set("remontée du bras vers la pose haute…"))
        r = subprocess.run(["bash", os.path.join(HOME, "roby_moveit_seq.sh"),
                            "--vel", "0.08", POSE_HAUTE],
                           env=ENV, capture_output=True, text=True)
        ok = "OK (code=1)" in (r.stdout or "")
        self._log("bras remonté — prêt à relancer" if ok else
                  f"⚠️ remontée NON confirmée : {(r.stdout or r.stderr or '')[-200:]}")
        self.root.after(0, lambda: self.etat.set(
            "arrêté — bras en pose haute, prêt à relancer" if ok else
            "arrêté — ⚠️ VÉRIFIER la position du bras"))
        self.root.after(0, lambda: self.lbl.configure(fg="#198754" if ok else "#dc3545"))
        self.root.after(0, lambda: self.b_start.configure(state="normal"))

    def on_close(self):
        if self.infer or self.guard:
            self._stop()
        self.root.destroy()


if __name__ == "__main__":
    r = tk.Tk(); Panel(r); r.mainloop()
