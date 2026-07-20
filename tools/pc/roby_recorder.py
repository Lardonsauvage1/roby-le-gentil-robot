#!/usr/bin/env python3
"""
roby_recorder.py — Enregistreur d'episodes (dataset imitation learning), sur le PC.

Enregistre les topics TELS QU'ILS ARRIVENT au PC (= exactement l'entree du reseau
de neurones) dans un bag mcap. Un bag par episode. Pilotable via la classe Recorder
(pour la future boucle oracle : rec.start(nom) / rec.stop()) ou en CLI pour tester.

- storage mcap (haut debit, images).
- SIGINT au sous-process 'ros2 bag record' -> fermeture PROPRE du mcap.
- rosbag2 adapte la QoS a celle des publishers (best_effort pour les images) par defaut.

Lancer avec /usr/bin/python3 (piege pyenv du PC) + ROS source + env DDS (domaine 42, cyclone).
"""
import argparse
import json
import os
import signal
import subprocess
import time

# Topics = ce que verra/produira le reseau (a l'entree du reseau, cote PC)
TOPICS = [
    "/head_camera/left/image_raw/compressed",    # observation : cam gauche
    "/head_camera/right/image_raw/compressed",   # observation : cam droite
    "/joint_states",                             # observation (etat) + ACTION (open-loop = consigne)
    "/gripper",                                  # action : pince
    "/head_lock",                                # action : verrou tete
    "/cube_pose",                                # etiquetage/debug SEULEMENT (pas une entree reseau)
]


class Recorder:
    def __init__(self, base_dir, topics=TOPICS):
        self.base = os.path.expanduser(base_dir)
        self.topics = topics
        self.proc = None
        os.makedirs(self.base, exist_ok=True)

    def start(self, name, meta=None):
        out = os.path.join(self.base, name)
        # stderr du recorder capture dans un log (avant : /dev/null => l'echec
        # 'output dir existe deja' etait SILENCIEUX et l'episode non enregistre).
        self._logf = open(out + ".rec.log", "w")
        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", out] + self.topics
        self.proc = subprocess.Popen(cmd, stdout=self._logf, stderr=subprocess.STDOUT)
        if meta is not None:
            # Fiche d'infos A COTE du bag : 'ros2 bag record' exige que <out> n'existe
            # pas encore, donc on ecrit <out>.meta.json (pas dans le dossier du bag).
            try:
                with open(out + ".meta.json", "w") as f:
                    json.dump(meta, f, indent=2, ensure_ascii=False)
            except Exception as e:
                print(f"  [meta] echec ecriture ({e})")
        return out

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)      # ferme proprement le mcap
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                self.proc.wait()
        self.proc = None
        if getattr(self, "_logf", None):
            try:
                self._logf.close()
            except Exception:
                pass
            self._logf = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="~/roby_datasets", help="repertoire de base des bags")
    ap.add_argument("--name", default=None, help="nom de l'episode (defaut: ep_HHMMSS)")
    ap.add_argument("--seconds", type=float, default=8.0, help="duree (mode test CLI)")
    args = ap.parse_args()

    name = args.name or ("ep_" + time.strftime("%H%M%S"))
    rec = Recorder(args.out)
    out = rec.start(name)
    print(f"REC -> {out}   ({args.seconds:.0f}s, {len(rec.topics)} topics)")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    rec.stop()
    print("STOP. Contenu du bag :\n")
    subprocess.run(["ros2", "bag", "info", out])


if __name__ == "__main__":
    main()
