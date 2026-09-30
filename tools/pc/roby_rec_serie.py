#!/usr/bin/env python3
"""Marque les N PROCHAINS essais enregistres par le panneau comme une serie liee.

    python3 ~/ros2_ws/tools/pc/roby_rec_serie.py --n 3 --note "raison a preciser"

Attend les fiches .run.json creees APRES son lancement et n'y ecrit qu'une fois l'essai
TERMINE (champ "fin" rempli par le panneau au STOP) : le panneau ne reecrit plus la fiche
ensuite, rien n'est ecrase. Ajoute "serie" au JSON et une section au .run.md, puis
s'arrete au N-ieme essai. Lecture seule vis-a-vis du robot.
"""
import argparse
import glob
import json
import os
import time

p = argparse.ArgumentParser()
p.add_argument("--n", type=int, default=3)
p.add_argument("--note", default="Essais lies - raison a preciser par Niels")
p.add_argument("--dir", default=os.path.expanduser(os.environ.get("ROBY_REC_DIR", "~/roby_datasets/rollouts")))
a = p.parse_args()

debut = time.time()
serie = "SERIE-" + time.strftime("%Y%m%d-%H%M")
deja = set(glob.glob(f"{a.dir}/*.run.json"))
fait = []
print(f"{serie} : attente de {a.n} essais enregistres dans {a.dir}", flush=True)
while len(fait) < a.n:
    for f in sorted(set(glob.glob(f"{a.dir}/*.run.json")) - deja - set(fait)):
        try:
            d = json.load(open(f, encoding="utf-8"))
        except Exception:
            continue                       # fiche en cours d'ecriture : on repassera
        if not d.get("fin"):
            continue                       # essai pas termine : le panneau reecrira la fiche
        rang = len(fait) + 1
        d["serie"] = {"id": serie, "rang": f"{rang}/{a.n}", "note": a.note,
                      "marque_le": time.strftime("%Y-%m-%d %H:%M:%S")}
        json.dump(d, open(f, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
        with open(f.replace(".run.json", ".run.md"), "a", encoding="utf-8") as fh:
            fh.write(f"\n## Série\n- `{serie}` — essai {rang}/{a.n} — {a.note}\n")
        fait.append(f)
        print(f"{time.strftime('%H:%M:%S')} {rang}/{a.n} marque : {os.path.basename(f)}", flush=True)
        if len(fait) >= a.n:
            break
    time.sleep(2)
print(f"{serie} complete.", flush=True)
