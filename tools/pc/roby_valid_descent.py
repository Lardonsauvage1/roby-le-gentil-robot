#!/usr/bin/env python3
"""Valide (SANS bouger le bras) le suivi de la descente droite sur toute la
table, pour w_ori=1.0 (avant) vs 0.2 (apres). Reproduit la logique de
roby_tool_pickup.straight() : depart au-dessus de R (IK top-down), on tient
l'orientation du depart, on descend verticalement, on mesure l'ecart max FK.
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~"))
import roby_tool_pickup as tp
import roby_oracle as o

STEP = tp.STEP
TOL = 0.010  # seuil du garde-fou (10 mm)


def descent_track(R_xyz, w_ori):
    """Ecart max (m) de la descente z_lift->z_pick a la verticale de R."""
    above = np.array([R_xyz[0], R_xyz[1], o._z_lift()])
    low = np.array([R_xyz[0], R_xyz[1], o._z_pick()])
    # IK du point d'approche (seed D, orientation top-down R_GRASP)
    japp = tp.dls(o.D_JOINTS, above, o.R_GRASP, w_ori=w_ori)
    if not o._in_limits(japp):
        return None
    keepR = tp.fkT(japp)[:3, :3]
    p0 = tp.fk_pos(japp)
    N = max(2, int(np.ceil(np.linalg.norm(low - p0) / STEP)))
    j = np.array(japp, float); track = 0.0
    for i in range(1, N + 1):
        wp = p0 + (i / N) * (low - p0)
        j = tp.dls(j, wp, keepR, w_ori=w_ori)
        if not o._in_limits(j):
            return None
        track = max(track, np.linalg.norm(tp.fk_pos(j) - wp))
    return track


def sweep(w_ori, rng):
    (xlo, xhi), (ylo, yhi) = o._sampling_bounds()
    ok = 0; fail = 0; unreach = 0; worst = 0.0
    for _ in range(400):
        x, y = rng.uniform(xlo, xhi), rng.uniform(ylo, yhi)
        if not o._zone_ok(x, y):
            continue
        t = descent_track((x, y, o._z_pick()), w_ori)
        if t is None:
            unreach += 1; continue
        worst = max(worst, t)
        if t <= TOL:
            ok += 1
        else:
            fail += 1
    tot = ok + fail
    rate = (100.0 * ok / tot) if tot else 0.0
    return ok, fail, unreach, worst, rate


def main():
    import random
    for w in (1.0, 0.2):
        rng = random.Random(1)  # meme tirage pour comparer a iso-points
        ok, fail, unreach, worst, rate = sweep(w, rng)
        print("w_ori=%.1f : descentes OK(<10mm)=%d  ECHEC(>10mm)=%d  hors-butee=%d  "
              "pire_suivi=%.1fmm  taux_reussite=%.0f%%"
              % (w, ok, fail, unreach, worst * 1000, rate))


if __name__ == "__main__":
    main()
