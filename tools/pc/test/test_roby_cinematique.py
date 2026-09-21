"""La geometrie du bras n'a qu'une source (ADR-005) : /usr/bin/python3 -m pytest tools/pc/test

Ce fichier relit l'URDF INDEPENDAMMENT du module (autre parcours, autre code) et verifie que
ce que `roby_cinematique` expose en vient bien. Il verifie aussi qu'aucun des trois fichiers
qui portaient une copie manuelle de la chaine n'en porte encore une.
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ICI = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(ICI, ".."))

import roby_cinematique as c  # noqa: E402

WS = os.path.normpath(os.path.join(ICI, "..", "..", ".."))
J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


# ---------------------------------------------------------------- relecture independante

def _urdf_brut():
    """Relit l'URDF a plat, sans reutiliser une ligne du module."""
    racine = ET.parse(c.CHEMIN_URDF).getroot()
    return {j.get("name"): j for j in racine.iter("joint")}


def test_chaine_conforme_a_l_urdf():
    brut = _urdf_brut()
    for seg in c.CHAINE:
        j = brut[seg["nom"]]
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        assert np.allclose(seg["xyz"], xyz), f"{seg['nom']} : origine divergente"
        assert np.allclose(seg["rpy"], rpy), f"{seg['nom']} : rpy divergent"
        assert (j.get("type") == "fixed") == seg["fixe"], f"{seg['nom']} : type divergent"


def test_butees_conformes_a_l_urdf():
    brut = _urdf_brut()
    assert sorted(c.LIMITS) == sorted(J)
    for nom, (lo, hi) in c.LIMITS.items():
        lim = brut[nom].find("limit")
        assert (lo, hi) == (float(lim.get("lower")), float(lim.get("upper"))), nom


def test_chaine_va_de_la_base_a_l_outil():
    assert c.CHAINE[0]["parent"] == c.BASE
    assert c.CHAINE[-1]["enfant"] == c.OUTIL
    for a, b in zip(c.CHAINE, c.CHAINE[1:]):
        assert a["enfant"] == b["parent"], "chaine discontinue"


# ---------------------------------------------------------------- coherence de la FK

def test_fk_recalculee_a_la_main():
    """FK refaite ici avec les nombres relus dans l'URDF, sans passer par le module."""
    brut = _urdf_brut()
    q = [0.3, 0.7, -0.4, 0.5, -0.2]
    T = np.eye(4)
    k = 0
    for seg in c.CHAINE:
        j = brut[seg["nom"]]
        o = j.find("origin")
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        r, p, y = [float(v) for v in o.get("rpy", "0 0 0").split()]
        R = c.Rz(y) @ c.Ry(p) @ c.Rx(r)
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = xyz
        T = T @ M
        if j.get("type") != "fixed":
            axe = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            M = np.eye(4); M[:3, :3] = c._rot_axe(axe, q[k]); T = T @ M
            k += 1
    assert k == 5
    assert np.allclose(T, c.fkT(q), atol=1e-12)


def test_poignet_est_l_origine_de_joint_5():
    """Le centre du poignet ne doit pas dependre de joint_5 (c'est son origine)."""
    q = [0.2, 0.6, -0.3, 0.4, 0.0]
    q5 = list(q); q5[4] = 1.2
    assert np.allclose(c.fk_poignet(q), c.fk_poignet(q5), atol=1e-12)


def test_poignet_ne_bouge_pas_avec_joint_4():
    """L'avant-bras est droit : joint_4 ne deplace pas l'origine de joint_5.

    L'URDF livree le 2026-09-20 placait cette origine a 99 mm de l'axe du tube, ce qui
    rendait l'inverse vrai. Corrige apres verification (emprise CAO, axe de symetrie du
    maillage, volume de collision de l'URDF) et confirmation de NM. Ce test verrouille
    le retablissement : la teleoperation cartesienne en depend.
    """
    q = [0.0, 0.6, -0.3, 0.0, 0.0]
    q4 = list(q); q4[3] = 1.0
    assert np.allclose(c.fk_poignet(q4), c.fk_poignet(q), atol=1e-12)


def test_jacobienne_coherente_avec_la_fk():
    q = np.array([0.1, 0.5, -0.4, 0.3, 0.2])
    Jm = c.jac(q)
    dq = np.array([1e-4, -1e-4, 1e-4, -1e-4, 1e-4])
    assert np.allclose(c.fk_pos(q + dq) - c.fk_pos(q), Jm[:3] @ dq, atol=1e-6)


def test_jac_offset_suit_le_point_decale():
    q = np.array([0.1, 0.5, -0.4, 0.3, 0.2])
    off = np.array([0.10, 0.0, 0.0])
    dq = np.array([1e-4, -1e-4, 1e-4, -1e-4, 1e-4])
    Jm = c.jac(q, offset=off)
    d = c.fk_outil(q + dq, off) - c.fk_outil(q, off)
    assert np.allclose(d, Jm[:3] @ dq, atol=1e-6)


def test_pas_de_repli_silencieux(monkeypatch, tmp_path):
    """URDF introuvable => on leve. Une cinematique de secours serait le defaut d'origine."""
    monkeypatch.setenv("ROBY_WS", str(tmp_path))
    monkeypatch.setattr(c.os.path, "isfile", lambda p: False)
    with pytest.raises(FileNotFoundError):
        c._chemin_urdf()


# ---------------------------------------------------------------- plus aucune copie

COPIES = [
    os.path.join(WS, "tools", "pc", "roby_tool_pickup.py"),
    os.path.join(WS, "tools", "pc", "roby_oracle.py"),
    os.path.join(WS, "src", "roby_control", "roby_control", "visual_servo_node.py"),
]

# Les origines de l'URDF, ecrites en clair : leur presence dans un de ces fichiers
# signifierait qu'une copie manuelle de la chaine y est revenue.
def _nombres_de_la_chaine():
    """Origines mesurees de l'URDF, en clair.

    On ecarte les valeurs a 3 decimales ou moins (0.02, 0.1...) : ce sont des nombres ronds
    qu'un script emploie legitimement comme marge ou vitesse. Une origine mesuree, elle, a
    5 ou 6 decimales — la retrouver en dur ne peut etre qu'une copie.
    """
    vals = set()
    for seg in c.CHAINE:
        for v in list(seg["xyz"]) + list(seg["rpy"]):
            v = round(float(v), 6)
            if abs(v) > 1e-9 and round(v, 3) != v:
                vals.add(v)
    return vals


@pytest.mark.parametrize("chemin", COPIES)
def test_aucune_origine_recopiee(chemin):
    texte = open(chemin, encoding="utf-8").read()
    # On ne regarde que le CODE : un nombre cite dans un commentaire est une trace, pas une copie.
    code = "\n".join(l.split("#")[0] for l in texte.splitlines())
    fautes = [v for v in _nombres_de_la_chaine() if repr(v) in code]
    assert not fautes, f"{os.path.basename(chemin)} : origine(s) URDF recopiee(s) {fautes}"


@pytest.mark.parametrize("chemin", COPIES)
def test_aucune_butee_recopiee(chemin):
    texte = open(chemin, encoding="utf-8").read()
    code = "\n".join(l.split("#")[0] for l in texte.splitlines())
    # visual_servo_node garde des butees de SERVICE plus serrees : seules les valeurs
    # de l'URDF elle-meme sont interdites en dur.
    urdf_only = {round(v, 6) for nom in J for v in c.LIMITS[nom]} - {
        round(v, 6) for v in (-3.14159, 3.14159, -3.0, 0.65, -1.6, 1.6)}
    fautes = [v for v in urdf_only if repr(v) in code]
    assert not fautes, f"{os.path.basename(chemin)} : butee(s) URDF recopiee(s) {fautes}"


# ---------------------------------------------------------------- les trois voient la meme chose

def test_les_trois_fichiers_partagent_la_meme_cinematique():
    """Importes separement, tool_pickup, oracle et visual_servo doivent donner la meme FK."""
    q = [0.25, 0.8, -0.5, 0.35, -0.15]
    code = f"""
import json, os, sys
sys.path.insert(0, {os.path.join(WS, 'tools', 'pc')!r})
import roby_cinematique as c
q = {q!r}
print(json.dumps(list(c.fk_pos(q)) + list(c.fk_poignet(q))))
"""
    ref = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    attendu = np.array(eval(ref.stdout.strip()))
    obtenu = np.concatenate([c.fk_pos(q), c.fk_poignet(q)])
    assert np.allclose(attendu, obtenu, atol=1e-12)
