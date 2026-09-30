"""Cinematique du bras, LUE DANS L'URDF — source unique (ADR-005, 2026-09-20).

Avant, la meme chaine etait recopiee a la main dans trois fichiers (roby_tool_pickup,
roby_oracle, visual_servo_node). Une mesure corrigee dans l'URDF ne les atteignait pas : la
garde a protege le bras avec une geometrie fausse de plus de 10 cm sans que rien ne le signale.
Ici, tout est lu dans l'URDF du workspace en service, et rien n'est recopie.

Ce module ne devine RIEN. S'il ne trouve pas l'URDF, il leve : une cinematique de repli
silencieuse serait exactement le defaut qu'on supprime.

    from roby_cinematique import fkT, fk_pos, fk_poignet, jac, LIMITS, CHAINE
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import numpy as np

BASE = "base_link"
OUTIL = "link_gripper"    # repere d'outil du projet : celui que protege le garde
J = [f"joint_{i}" for i in range(1, 6)]


def _chemin_urdf() -> str:
    """URDF du workspace en service (ROBY_WS), sinon celle du paquet installe."""
    candidats = []
    ws = os.environ.get("ROBY_WS")
    if ws:
        candidats.append(os.path.join(ws, "src", "neuroneimitationcarote_description",
                                      "urdf", "robot.urdf.xacro"))
    # Le paquet a cote de ce fichier (tools/pc/../../src/...) : meme copie du depot.
    ici = os.path.dirname(os.path.realpath(__file__))
    candidats.append(os.path.normpath(os.path.join(
        ici, "..", "..", "src", "neuroneimitationcarote_description", "urdf", "robot.urdf.xacro")))
    try:
        from ament_index_python.packages import get_package_share_directory
        candidats.append(os.path.join(get_package_share_directory(
            "neuroneimitationcarote_description"), "urdf", "robot.urdf.xacro"))
    except Exception:
        pass
    for c in candidats:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        "URDF introuvable (ADR-005 : la geometrie n'a qu'une source). Cherche : "
        + ", ".join(candidats))


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def H(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _rpy(r, p, y):
    """Convention URDF : angles fixes XYZ, soit Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return Rz(y) @ Ry(p) @ Rx(r)


def _rot_axe(axe, angle):
    """Rotation d'`angle` autour d'un axe quelconque (Rodrigues) : l'URDF le declare."""
    k = np.asarray(axe, float)
    n = float(np.linalg.norm(k))
    if n < 1e-12:
        return np.eye(3)
    k = k / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def _lire(chemin):
    """Chaine BASE -> OUTIL telle que l'URDF la declare, dans l'ordre."""
    racine = ET.parse(chemin).getroot()
    par_parent = {}
    for j in racine.iter("joint"):
        parent = j.find("parent")
        enfant = j.find("child")
        if parent is None or enfant is None:
            continue
        par_parent.setdefault(parent.get("link"), []).append((j, enfant.get("link")))

    chaine, limites, corps = [], {}, BASE
    vus = set()
    while corps != OUTIL:
        suite = None
        for j, enfant in par_parent.get(corps, []):
            # On suit la branche qui mene a l'outil ; un embranchement mort est ignore.
            if _mene_a(par_parent, enfant, OUTIL):
                suite = (j, enfant)
                break
        if suite is None:
            raise ValueError(f"URDF : aucune chaine de {corps} vers {OUTIL} dans {chemin}")
        j, enfant = suite
        if enfant in vus:
            raise ValueError(f"URDF : boucle sur {enfant}")
        vus.add(enfant)
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
        axe_el = j.find("axis")
        axe = [float(v) for v in axe_el.get("xyz").split()] if axe_el is not None else None
        fixe = j.get("type") == "fixed"
        chaine.append({"nom": j.get("name"), "parent": corps, "enfant": enfant,
                       "xyz": np.array(xyz), "rpy": np.array(rpy),
                       "axe": None if fixe else np.array(axe if axe else [0, 0, 1.0]),
                       "fixe": fixe})
        lim = j.find("limit")
        if lim is not None and not fixe:
            limites[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        corps = enfant
    return chaine, limites


def _mene_a(par_parent, corps, cible):
    if corps == cible:
        return True
    return any(_mene_a(par_parent, enfant, cible) for _, enfant in par_parent.get(corps, []))


CHEMIN_URDF = _chemin_urdf()
CHAINE, LIMITS = _lire(CHEMIN_URDF)
# Index des articulations mobiles, dans l'ordre de la chaine : joint_1..joint_5.
MOBILES = [i for i, s in enumerate(CHAINE) if not s["fixe"]]
NOMS = [CHAINE[i]["nom"] for i in MOBILES]
if NOMS != J:
    raise ValueError(f"URDF : articulations attendues {J}, lues {NOMS} dans {CHEMIN_URDF}")
# Dernier segment mobile = joint_5 : son origine est le CENTRE DU POIGNET.
_FIN_POIGNET = MOBILES[-1] + 1


def _parcours(q, jusqu_a):
    T = np.eye(4)
    k = 0
    for s in CHAINE[:jusqu_a]:
        T = T @ H(_rpy(*s["rpy"]), s["xyz"])
        if not s["fixe"]:
            T = T @ H(_rot_axe(s["axe"], float(q[k])), [0, 0, 0])
            k += 1
    return T


def fkT(q):
    """Transformation BASE -> link_gripper (le point que protege le garde)."""
    return _parcours(q, len(CHAINE))


def fk_pos(q):
    return fkT(q)[:3, 3]


def fk_poignet(q):
    """CENTRE DU POIGNET : origine de la derniere articulation mobile (joint_5).

    ⚠️ Depuis la mesure du 2026-09-20, cette origine n'est plus sur l'axe de roulis de joint_4 :
    tourner joint_4 la deplace. Ne plus supposer qu'elle ne depend que de joint_1..3.
    """
    return _parcours(q, _FIN_POIGNET)[:3, 3]


def rotvec(Rm):
    ang = np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))
    if ang < 1e-8:
        return np.zeros(3)
    return ang / (2 * np.sin(ang)) * np.array(
        [Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])


def fk_outil(q, offset=None):
    """Position d'un point solidaire de l'outil, exprime dans le repere link_gripper.

    `offset` est un reglage de COMMANDE (pointe d'outil, cube suivi...), pas une grandeur
    de l'URDF : il reste chez l'appelant, et seule la chaine vient d'ici.
    """
    T = fkT(q)
    if offset is None:
        return T[:3, 3]
    return T[:3, 3] + T[:3, :3] @ np.asarray(offset, float)


def jac(q, eps=1e-5, offset=None):
    """Jacobienne 6 x 5 (position puis rotation), par differences finies.

    Au point `link_gripper` par defaut ; `offset` la deplace sur un point solidaire de
    l'outil (voir fk_outil).
    """
    T0 = fkT(q)
    R0 = T0[:3, :3]
    p0 = fk_outil(q, offset)
    Jm = np.zeros((6, len(J)))
    for k in range(len(J)):
        qq = np.array(q, float)
        qq[k] += eps
        T1 = fkT(qq)
        p1 = T1[:3, 3] if offset is None else T1[:3, 3] + T1[:3, :3] @ np.asarray(offset, float)
        Jm[:3, k] = (p1 - p0) / eps
        Jm[3:, k] = R0 @ rotvec(R0.T @ T1[:3, :3]) / eps
    return Jm


def jac_poignet(q, axes=None, eps=1e-6):
    """Jacobienne en position du centre du poignet (3 x n), colonnes `axes` (defaut : toutes)."""
    axes = list(range(len(J))) if axes is None else list(axes)
    p0 = fk_poignet(q)
    Jm = np.zeros((3, len(axes)))
    for c, i in enumerate(axes):
        qq = np.array(q, float)
        qq[i] += eps
        Jm[:, c] = (fk_poignet(qq) - p0) / eps
    return Jm


def resume():
    """Une ligne par segment : de quoi verifier a l'oeil ce qui a ete lu."""
    lignes = [f"URDF : {CHEMIN_URDF}"]
    for s in CHAINE:
        axe = "fixe" if s["fixe"] else f"axe {np.round(s['axe'], 3).tolist()}"
        lignes.append(f"  {s['nom']:14s} {s['parent']:12s} -> {s['enfant']:12s} "
                      f"xyz {np.round(s['xyz'], 6).tolist()} rpy {np.round(s['rpy'], 6).tolist()} {axe}")
    return "\n".join(lignes)


if __name__ == "__main__":
    print(resume())
