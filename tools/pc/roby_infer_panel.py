#!/usr/bin/env python3
"""Panneau minimal pour lancer/arrêter le modèle sur le vrai bras (2026-09-07).

Un seul geste utile : STOP. Il coupe l'inférence, laisse le garde figer la consigne,
puis ramène le bras à la pose de départ haute — celle sur laquelle le modèle a été
entraîné (médiane des débuts d'épisode du dataset) — donc prêt à relancer.

Le garde `roby_guard.py` est lancé/arrêté avec le panneau : l'inférence publie vers
/guard/joint_trajectory, rien n'atteint le bras sans lui.
"""
import atexit, json, os, signal, subprocess, sys, threading, time
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

# --- Enregistrement des essais (case "Enregistrer les essais" du panneau) ----------
# Un bag MCAP par essai, du clic DEMARRER au STOP (la remontee n'est pas enregistree),
# nomme d'apres le modele et le mode, avec une fiche .run.json/.run.md ecrite AU
# LANCEMENT (modele et reglages reels, pas deduits apres coup). Le resultat de l'essai
# est laisse "a renseigner" : il est observe par l'operateur, jamais deduit.
REC_DIR = os.path.expanduser(os.environ.get("ROBY_REC_DIR", "~/roby_datasets/rollouts"))
REC_CPUS = os.environ.get("ROBY_REC_CPUS", "12-19")   # E-cores : ne rien voler au modele
REC_TOPICS = [
    "/head_camera/left/image_raw/compressed", "/head_camera/right/image_raw/compressed",
    "/joint_states", "/roby_infer/action", "/roby_infer/gripper_raw", "/roby_infer/status",
    "/guard/joint_trajectory", "/guard/gripper",
]


def _json_ou_vide(chemin):
    try:
        with open(chemin, encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return {}


def details_modele(model):
    """Identite du modele lue sur le disque (config.json / train_config.json)."""
    cfg = _json_ou_vide(os.path.join(model, "config.json"))
    tcfg = _json_ou_vide(os.path.join(model, "train_config.json"))
    rel = model.split("/outputs/", 1)[-1].split("/")
    entrainement = rel[0] if rel else "?"
    checkpoint = os.path.basename(os.path.dirname(model.rstrip("/")))
    inp, out = cfg.get("input_features", {}), cfg.get("output_features", {})
    etat = inp.get("observation.state", {}).get("shape", ["?"])[0]
    action = out.get("action", {}).get("shape", ["?"])[0]
    return {
        "chemin": model,
        "entrainement": entrainement,
        "checkpoint": checkpoint,
        "corpus": tcfg.get("dataset", {}).get("repo_id"),
        "etat_action": f"{etat}D/{action}D",
        "cameras": [k.split(".")[-1] for k in inp if "image" in k],
        "reprise_de": tcfg.get("policy", {}).get("pretrained_path"),
        "export_openvino_present": any(os.path.exists(os.path.join(d, "unet_ov.xml"))
                                       for d in (model, os.path.dirname(model.rstrip("/")))),
    }


def _slug(s):
    return "".join(c if c.isalnum() or c in "-_." else "-" for c in s)


class Panel:
    def __init__(self, root):
        self.root = root
        self.infer = None
        self.guard = None
        self.rec = None          # processus ros2 bag record de l'essai en cours
        # Arret demande pendant le demarrage (fenetre fermee, STOP, signal) : le fil de
        # demarrage ne doit plus rien lancer. Sans ce drapeau, fermer la fenetre pendant
        # les 4 s d'attente du garde laissait le modele se lancer ensuite, ORPHELIN
        # (revue du 2026-09-13).
        self._arret_demande = False
        self._verrou = threading.Lock()
        self.rec_base = None     # chemin de l'essai enregistre (sans extension)
        self.run = None          # fiche de l'essai (ecrite au lancement, completee au STOP)
        root.title("Roby — modèle sur le bras")
        root.geometry("560x520")

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

        # --- Enregistrement : s'applique au PROCHAIN demarrage, reste coche ensuite --
        rf = ttk.Frame(root); rf.pack(pady=(2, 0))
        self.rec_var = tk.BooleanVar(value=os.environ.get("ROBY_REC", "") in ("1", "oui", "true"))
        tk.Checkbutton(rf, text="⏺ Enregistrer les essais (caméras + modèle) — au prochain DÉMARRER",
                       variable=self.rec_var, font=("TkDefaultFont", 10, "bold")).pack(side="left")
        self.rec_etat = tk.StringVar(value="")
        tk.Label(root, textvariable=self.rec_etat, fg="#dc3545",
                 font=("TkDefaultFont", 9, "bold"), wraplength=540).pack()

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
            # Message fige ICI : Python efface `e` a la sortie du bloc except, une
            # lambda qui le lit plus tard levait NameError (message jamais affiche).
            msg = f"affichage pince indisponible : {e}"
            self.root.after(0, lambda: self._log(msg))
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
            msg = f"écoute pince arrêtée : {e}"
            self.root.after(0, lambda: self._log(msg))

    def _log(self, m):
        ligne = f"[{time.strftime('%H:%M:%S')}] {m}"
        try:
            self.log.insert("end", ligne + "\n")
            self.log.see("end")
        except Exception:        # fenetre deja detruite (arret d'urgence, sortie)
            print(ligne, flush=True)

    # ------------------------------------------------------------------ start
    def on_start(self):
        if self.infer:
            return
        autre = _garde_etranger(self.guard)
        if autre:
            self.etat.set(f"⚠️ un autre garde tourne déjà (PID {autre}) : téléopération ? "
                          "L'arrêter d'abord — deux gardes relaient le même topic.")
            return
        self._arret_demande = False
        self.b_start.configure(state="disabled")
        self.etat.set("démarrage du garde puis du modèle…")
        enregistrer = bool(self.rec_var.get())     # lu ici : tkinter reste dans son fil
        threading.Thread(target=self._start, args=(enregistrer,), daemon=True).start()

    # ------------------------------------------------------------ enregistrement
    def _rec_start(self, cmd):
        """Lance le bag de l'essai et ecrit sa fiche AVANT le garde et le modele."""
        mode = ("RTC" if "--rtc" in cmd else
                "async-offset%s" % cmd[cmd.index("--exec-offset") + 1] if "--exec-offset" in cmd
                else "async")
        mod = details_modele(MODEL)
        debut = time.strftime("%Y%m%d_%H%M%S")
        nom = f"rollout_{debut}_{_slug(mod['entrainement'])}-{_slug(mod['checkpoint'])}_{mode}"
        os.makedirs(REC_DIR, exist_ok=True)
        self.rec_base = os.path.join(REC_DIR, nom)

        def opt(o, defaut=None):
            return cmd[cmd.index(o) + 1] if o in cmd else defaut

        self.run = {
            "essai": nom,
            "debut": time.strftime("%Y-%m-%d %H:%M:%S"),
            "modele": mod,
            "inference": {
                "mode": mode,
                "rtc_delay": opt("--rtc-delay", "4 (defaut)") if mode == "RTC" else None,
                "rtc_guide": opt("--rtc-guide", "4 (defaut)") if mode == "RTC" else None,
                "exec_offset": opt("--exec-offset"),
                "accelerateur": opt("--igpu", "CPU"),
                "cpus_modele": INFER_CPUS or "non epingle",
                "omp_num_threads": ENV.get("OMP_NUM_THREADS"),
                "hz": opt("--hz"), "pas_de_diffusion": opt("--steps"), "w_ori": opt("--w-ori"),
                "j3_scale": ENV.get("ROBY_J3_SCALE"), "camera_modele": ENV.get("ROBY_INFER_CAM"),
                "commande": " ".join(cmd),
            },
            "pose_de_remontee": POSE_HAUTE,
            "fin": None, "duree_s": None, "bag_ferme_proprement": None,
            "resultat": "A RENSEIGNER (observe par l'operateur, jamais deduit)",
            "remarques": [],
        }
        self._rec_ecrire_fiche()
        self.rec = subprocess.Popen(
            ["taskset", "-c", REC_CPUS, "ros2", "bag", "record", "-s", "mcap",
             "-o", self.rec_base, "--topics", *REC_TOPICS],
            env=ENV, stdout=open(self.rec_base + ".bag.log", "w"),
            stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        self._log(f"⏺ enregistrement : {nom}")
        self.root.after(0, lambda: self.rec_etat.set(f"● ENREGISTREMENT  {nom}"))

    def _rec_stop(self):
        """Ferme le bag PROPREMENT. SIGTERM et non SIGINT : lance depuis un process
        sans controle de taches, rosbag2 herite de SIGINT ignore (vecu 2026-09-13)."""
        if not self.rec:
            return
        t0 = time.time()
        try:
            os.killpg(os.getpgid(self.rec.pid), signal.SIGTERM)
            self.rec.wait(timeout=10)
            propre = True
        except Exception:
            propre = False
            try: os.killpg(os.getpgid(self.rec.pid), signal.SIGKILL)
            except Exception: pass
            self.run["remarques"].append("enregistreur tue apres 10 s : bag a reindexer "
                                         "(ros2 bag reindex -s mcap <dossier>)")
        self.run["fin"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.run["duree_s"] = round(t0 - time.mktime(time.strptime(self.run["debut"], "%Y-%m-%d %H:%M:%S")), 1)
        self.run["bag_ferme_proprement"] = propre and os.path.exists(
            os.path.join(self.rec_base, "metadata.yaml"))
        self._rec_ecrire_fiche()
        self._log(("⏹ enregistrement fermé : " if propre else "⚠️ enregistrement tué : ")
                  + os.path.basename(self.rec_base))
        self.root.after(0, lambda: self.rec_etat.set(""))
        self.rec = None

    def _rec_ecrire_fiche(self):
        r = self.run
        with open(self.rec_base + ".run.json", "w", encoding="utf-8") as fh:
            json.dump(r, fh, ensure_ascii=False, indent=2)
        m, i = r["modele"], r["inference"]
        lignes = [
            f"# {r['essai']}", "",
            f"**Résultat : {r['resultat']}**", "",
            f"- Début {r['debut']} · fin {r['fin'] or '—'} · durée {r['duree_s'] or '—'} s"
            f" · bag fermé proprement : {r['bag_ferme_proprement']}", "",
            "## Modèle",
            f"- `{m['entrainement']}` / checkpoint `{m['checkpoint']}` — {m['etat_action']},"
            f" caméra(s) {', '.join(m['cameras']) or '?'}, corpus `{m['corpus']}`",
            f"- Chemin : `{m['chemin']}`",
            f"- Reprise de : `{m['reprise_de']}` · export OpenVINO présent : {m['export_openvino_present']}", "",
            "## Inférence (relevée au lancement)",
            f"- Mode {i['mode']}" + (f" (gel {i['rtc_delay']}, guidage {i['rtc_guide']})" if i["mode"] == "RTC" else "")
            + (f" · exec-offset {i['exec_offset']}" if i["exec_offset"] else ""),
            f"- Accélérateur {i['accelerateur']} · CPU {i['cpus_modele']} · OMP {i['omp_num_threads']}",
            f"- {i['hz']} Hz · {i['pas_de_diffusion']} pas de diffusion · w_ori {i['w_ori']}"
            f" · j3_scale {i['j3_scale']} · caméra {i['camera_modele']}",
            f"- Commande : `{i['commande']}`", "",
            "## Remarques", *([f"- {x}" for x in r["remarques"]] or ["- aucune"]), "",
            f"Bag : `{self.rec_base}/` · journal `{self.rec_base}.bag.log`", "",
        ]
        with open(self.rec_base + ".run.md", "w", encoding="utf-8") as fh:
            fh.write("\n".join(lignes))

    def _start(self, enregistrer=False):
        # La commande du modele est construite AVANT tout lancement : la fiche de
        # l'essai enregistre les reglages reels, et le bag demarre des le clic.
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
        if enregistrer:
            try:
                self._rec_start(cmd)
            except Exception as e:       # un enregistrement rate ne doit pas bloquer l'essai
                self.rec = None
                self._log(f"⚠️ enregistrement NON lancé : {e}")
        with self._verrou:
            if self._arret_demande:
                self._log("arrêt demandé pendant le démarrage : rien n'est lancé")
                return
            if not self.guard or self.guard.poll() is not None:
                self.guard = subprocess.Popen(
                    ["bash", os.path.join(HOME, "roby_guard.sh")],
                    env=ENV, stdout=open("/tmp/guard.log", "w"),
                    stderr=subprocess.STDOUT, preexec_fn=os.setsid)
                self._log(f"garde lancé (PID {self.guard.pid})")
                attendre = True
            else:
                attendre = False
        if attendre:
            time.sleep(4)
        # Les sorties partaient dans /dev/null : un plantage du modele ou un refus du
        # garde etaient donc totalement muets, le panneau affichant "modele lance" sans
        # rien verifier. Vecu le 2026-09-10 : "pourquoi le modele ne fait rien ?" sans
        # aucune trace nulle part. On les ecrit desormais dans des fichiers relisibles.
        with self._verrou:
            if self._arret_demande:
                self._log("arrêt demandé pendant le démarrage : modèle NON lancé")
                return
            self.infer = subprocess.Popen(cmd, env=ENV, stdout=open("/tmp/infer.log", "w"),
                                          stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        self._log(f"modèle lancé (PID {self.infer.pid}) — le bras est AUTONOME")
        self.root.after(1000, self._surveiller_modele)
        self.root.after(0, lambda: self.etat.set("⚠️ MODÈLE EN COURS — le bras bouge seul"))
        self.root.after(0, lambda: self.lbl.configure(fg="#dc3545"))
        self.root.after(0, lambda: self.b_stop.configure(state="normal"))

    # ------------------------------------------------------------------- stop
    def on_stop(self):
        self.b_stop.configure(state="disabled")
        self.etat.set("arrêt en cours…")
        self.lbl.configure(fg="#b8860b")
        threading.Thread(target=self._stop, daemon=True).start()

    def _surveiller_modele(self):
        """Le modele s'est-il arrete tout seul ? Sans cela, un plantage laissait le panneau
        afficher « MODELE EN COURS » (revue du 2026-09-13). Aucun mouvement n'est lance
        ici : le garde reste en place (le bras tient), l'operateur decide avec STOP."""
        p = self.infer
        if p is None or self._arret_demande:
            return
        code = p.poll()
        if code is None:
            self.root.after(1000, self._surveiller_modele)
            return
        self._log(f"⚠️ le modèle s'est ARRÊTÉ tout seul (code {code}) — voir /tmp/infer.log")
        self.etat.set("⚠️ MODÈLE ARRÊTÉ (plantage ?) — bras tenu par le garde ; STOP pour finir")
        self.lbl.configure(fg="#dc3545")

    def _couper_processus(self):
        """Arrete modele, garde et enregistrement. AUCUN mouvement (pas de remontee) :
        le bras garde sa derniere consigne. Sert a STOP (avant la remontee) et a l'arret
        d'urgence du panneau (signal, sortie)."""
        with self._verrou:
            self._arret_demande = True
            procs = (("modèle", self.infer), ("garde", self.guard))
            self.infer = self.guard = None
        for nom, p in procs:
            if p and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=5)
                except Exception:
                    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception: pass
                self._log(f"{nom} arrêté")
        try:
            self._rec_stop()
        except Exception as e:
            self._log(f"⚠️ fermeture de l'enregistrement : {e}")

    def _stop(self):
        self._couper_processus()     # l'essai s'arrete ici : la remontee n'est pas enregistree
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


def _garde_etranger(le_notre):
    """PID d'un roby_guard.py qui n'est pas celui du panneau, ou None.

    Deux gardes relaient le meme topic : un gel dans l'un est contourne par l'autre.
    Motif ancre sur un vrai processus python (pas un shell qui cite le nom)."""
    r = subprocess.run(["pgrep", "-f", r"^[^ ]*python[^ ]* [^ ]*roby_guard\.py"],
                       capture_output=True, text=True)
    notre = None
    if le_notre is not None and le_notre.poll() is None:
        notre = str(le_notre.pid)
    for pid in r.stdout.split():
        if pid == notre:
            continue
        try:   # le python du garde lance par NOTRE roby_guard.sh a pour parent ce bash
            ppid = open(f"/proc/{pid}/stat").read().split()[3]
        except OSError:
            continue
        if ppid != notre:
            return pid
    return None


if __name__ == "__main__":
    r = tk.Tk()
    panneau = Panel(r)

    # Le modele et le garde tournent dans leur propre groupe de processus : ils ne
    # recoivent ni le Ctrl-C ni la fermeture du terminal du panneau. Avant le
    # 2026-09-13, un panneau tue ainsi laissait le modele piloter le bras, sans bouton
    # STOP. On les arrete donc sur signal et a la sortie -- sans remontee (aucun
    # mouvement lance par un arret anormal). Un SIGKILL du panneau reste non couvert.
    def _arret_urgence(signum, _frame):
        panneau._couper_processus()
        try:
            r.destroy()
        except Exception:
            pass
        sys.exit(128 + signum)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _arret_urgence)
    atexit.register(panneau._couper_processus)

    def _battement():            # rend la main a Python : les signaux sont traites
        r.after(250, _battement)
    _battement()
    r.mainloop()
