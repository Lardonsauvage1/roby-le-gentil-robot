#!/usr/bin/env python3
"""roby_plot_loss.py — trace la loss d'entrainement (echelle log) + le learning rate.

Pourquoi un script plutot qu'un bricolage : les graphes precedents etaient refaits
a la main a chaque fois. Et surtout le LR sur le second axe est ce qui permet de
VERIFIER d'un coup d'oeil que le scheduler demande est bien celui qui tourne --
piege vecu deux fois : `constant_with_warmup` remplace silencieusement par `cosine`,
parce que LeRobot lit `policy.scheduler_name` et non le bloc `scheduler`.

Le numero de pas n'est PAS lu dans le texte : le log abrege ("step:26K"), ce qui
perd la precision. Il est reconstruit a partir de `--log-freq` (50 par defaut, la
valeur de la config), une ligne etant emise tous les log_freq pas.

Usage :
  roby_plot_loss.py <log> [<log2> ...] [--out fig.png] [--log-freq 50]
                    [--labels "phase 1,cooldown"] [--titre "..."]
"""
import argparse
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

MOTIF = re.compile(r"loss:([0-9.]+).*?lr:([0-9.e+-]+)")


def lire(chemin, log_freq):
    """-> (pas, loss, lr). Les lignes tqdm sont ignorees, seules les lignes INFO comptent."""
    pas, loss, lr = [], [], []
    with open(os.path.expanduser(chemin), errors="ignore") as f:
        for ligne in f.read().replace("\r", "\n").split("\n"):
            m = MOTIF.search(ligne)
            if m:
                loss.append(float(m.group(1)))
                lr.append(float(m.group(2)))
                pas.append(log_freq * len(loss))
    return np.array(pas), np.array(loss), np.array(lr)


def fenetre(v):
    """Fenetre de lissage adaptee : sur un run qui demarre on n'a que quelques
    dizaines de points, une fenetre fixe de 20 masquerait tout le debut."""
    return max(3, min(20, len(v) // 8))


def lisse(v, n):
    if len(v) < n:
        return v
    return np.convolve(v, np.ones(n) / n, mode="valid")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default=os.path.expanduser("~/loss.png"))
    ap.add_argument("--log-freq", type=int, default=50)
    ap.add_argument("--labels", default="")
    ap.add_argument("--titre", default="Loss d'entrainement")
    a = ap.parse_args()
    labels = [x.strip() for x in a.labels.split(",")] if a.labels else \
             [os.path.basename(x) for x in a.logs]

    fig, ax = plt.subplots(figsize=(11, 6))
    ax2 = ax.twinx()
    decalage = 0
    couleurs = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    for i, lg in enumerate(a.logs):
        p, l, r = lire(lg, a.log_freq)
        if len(p) == 0:
            print(f"  (aucune ligne de loss dans {lg})", file=sys.stderr)
            continue
        p = p + decalage
        c = couleurs[i % len(couleurs)]
        ax.plot(p, l, color=c, alpha=0.25, lw=0.8)
        n = fenetre(l)
        ax.plot(p[n - 1:], lisse(l, n), color=c, lw=1.8,
                label=f"{labels[i]} (loss finale {l[-1]:.4f})")
        ax2.plot(p, r, color=c, ls="--", lw=1.0, alpha=0.6)
        if i == 0:
            decalage = p[-1]
        print(f"  {labels[i]:20s} {len(p):5d} points | loss {l[0]:.4f} -> {l[-1]:.4f} "
              f"| lr {r[0]:.1e} -> {r[-1]:.1e}")

    ax.set_yscale("log")
    ax.set_xlabel("pas d'entrainement")
    ax.set_ylabel("loss (echelle log)")
    ax2.set_ylabel("learning rate (pointilles)")
    ax2.set_yscale("log")
    ax.grid(True, which="both", alpha=0.25)
    ax.set_title(a.titre)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print(f"  -> {a.out}")


if __name__ == "__main__":
    sys.exit(main())
