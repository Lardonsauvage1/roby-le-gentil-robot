#!/usr/bin/env python3
"""
roby_goto.py — Amene la pince EN SURVOL au-dessus d'un point (x,y) de la table,
pince ouverte, pour poser un objet dessous a la bonne place (ex : cone a R avant
un rejeu fidele). Reutilise l'IK grasp-azimut + le transit MoveIt de l'oracle.

Usage : python3 roby_goto.py X Y [--above 0.08] [--go]
  sans --go : DRY (calcule, ne bouge pas).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.expanduser("~"))
import numpy as np                       # noqa: E402
import roby_oracle as o                  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--above", type=float, default=0.08, help="hauteur de survol au-dessus de la cible (m)")
    ap.add_argument("--go", action="store_true", help="requis pour bouger le bras")
    args = ap.parse_args()

    zc = o._z_pick(args.x, args.y)                 # z corrige (inclinaison) de la cible
    hover = (args.x, args.y, zc + args.above)
    print(f"cible=({args.x:.3f},{args.y:.3f})  z_table_corrige={zc:.3f}  survol z={hover[2]:.3f}")

    mode = "real" if args.go else "plan"
    m = o.Motion(mode, vel=0.15)
    j = m.ik(hover)
    if j is None:
        print("IK hors butee pour ce survol."); return 1
    print("IK survol OK, outil vertical (grasp-azimut).")
    if not args.go:
        print("DRY : ajoute --go pour amener la pince en survol."); return 0

    m.gripper(close=False)                          # pince ouverte
    if not m.move_free("survol", hover):
        print("Transit MoveIt echoue."); return 1
    print("Pince en survol au-dessus de la cible (ouverte). Pose le cone dessous.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
