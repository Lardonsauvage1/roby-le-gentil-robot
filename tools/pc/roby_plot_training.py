#!/usr/bin/env python3
"""Trace les courbes d'entrainement du balayage de tailles.

Lit les <sortie>/<taille>/metrics.csv produits par roby_train_xpu.py et sort un PNG
comparant les modeles : train-loss (lineaire + log) et norme du gradient.

/!\\ Il n'y a PAS de val-loss : l'entrainement se fait sur les 111 episodes, sans jeu de
validation (choix assume, comme la recette de reference). Ne pas confondre la courbe de
train avec une mesure de generalisation.

Lancer : ~/ipex_test_venv/bin/python ~/roby_plot_training.py --out ~/lerobot-experiments/outputs/cart_sweep_20260727
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def lire(p):
    steps, loss, grad = [], [], []
    with open(p) as f:
        for r in csv.DictReader(f):
            steps.append(int(r["step"]))
            loss.append(float(r["loss"]))
            grad.append(float(r["grad_norm"]))
    return steps, loss, grad


def lisser(v, k=51):
    """Moyenne glissante centree, pour rendre les courbes lisibles sous le bruit."""
    if len(v) < k or k < 2:
        return v
    out, acc = [], 0.0
    from collections import deque
    d = deque()
    for x in v:
        d.append(x)
        acc += x
        if len(d) > k:
            acc -= d.popleft()
        out.append(acc / len(d))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True, help="dossier du balayage")
    p.add_argument("--png", default=None)
    p.add_argument("--lissage", type=int, default=51)
    a = p.parse_args()

    root = Path(a.out).expanduser()
    runs = []
    for d in sorted(root.iterdir()):
        f = d / "metrics.csv"
        if f.is_file():
            donnees = lire(f)
            if len(donnees[0]) < 2:
                print(f"  (ignore {d.name} : run en cours, pas encore de donnees)")
                continue
            info = {}
            ji = d / "train_info.json"
            if ji.is_file():
                info = json.loads(ji.read_text())
            runs.append((d.name, donnees, info))
    if not runs:
        raise SystemExit(f"aucun metrics.csv sous {root}")

    fig, axes = plt.subplots(1, 3, figsize=(19, 5.2))
    couleurs = plt.cm.viridis([0.05, 0.35, 0.62, 0.88])

    for i, (nom, (st, lo, gr), info) in enumerate(runs):
        c = couleurs[i % len(couleurs)]
        lab = f"{nom}"
        if info.get("params_M"):
            lab += f" ({info['params_M']:.0f}M)"
        axes[0].plot(st, lisser(lo, a.lissage), color=c, label=lab, lw=1.6)
        axes[1].plot(st, lisser(lo, a.lissage), color=c, label=lab, lw=1.6)
        axes[2].plot(st, lisser(gr, a.lissage), color=c, label=lab, lw=1.6)

    axes[0].set_title("Train loss")
    axes[1].set_title("Train loss (echelle log)")
    axes[1].set_yscale("log")
    axes[2].set_title("Norme du gradient (avant ecretage a 10)")
    axes[2].axhline(10.0, color="crimson", ls="--", lw=1, label="seuil d'ecretage")

    for ax in axes:
        ax.set_xlabel("pas d'entrainement")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=9)

    fig.suptitle("Balayage de tailles — Diffusion Policy cartesienne, 3 h par modele "
                 "(LR constant 1e-4, PAS de validation)", fontsize=12)
    fig.tight_layout()
    png = Path(a.png).expanduser() if a.png else root / "courbes_entrainement.png"
    fig.savefig(png, dpi=130)
    print(f"graphe ecrit : {png}")

    print(f"\n{'modele':8s} {'params':>8s} {'pas':>8s} {'loss fin':>10s} {'grad fin':>9s}")
    for nom, (st, lo, gr), info in runs:
        n = max(len(lo) // 20, 1)
        print(f"{nom:8s} {info.get('params_M', 0):7.1f}M {st[-1]:8d} "
              f"{sum(lo[-n:])/n:10.5f} {sum(gr[-n:])/n:9.3f}")


if __name__ == "__main__":
    main()
