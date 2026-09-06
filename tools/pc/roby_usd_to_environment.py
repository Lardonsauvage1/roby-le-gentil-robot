#!/usr/bin/env python3
"""Convertit une scene de collision USD en environnement roby_environments (YAML).

Ecrit pour importer le plan 3D de la cuisine produit par l'equipe Le NIC
(Sweet Home 3D -> Isaac), dont la geometrie de collision est faite de boites.

Ce que fait la conversion :
  - lit les `def Cube` du fichier USDA (centre = xformOp:translate, dimensions =
    xformOp:scale car `size = 1`, orientation = xformOp:orient en quaternion WXYZ) ;
  - passe du repere CUISINE au repere BASE ROBOT, en appliquant l'inverse de la pose
    du robot dans la scene ;
  - ne garde que ce qui est a portee (rayon reglable) : inutile de charger les murs
    du fond dans la scene de planification ;
  - ecrit un YAML au format roby_environments (frame `world` = base du robot).

    python3 roby_usd_to_environment.py <collision.usda> --pose <placement.usda|json> \
        --sortie environnements/cuisine.yaml [--rayon 2.5]
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys

import yaml

MIN_EPAISSEUR = 0.01  # m

# Couleurs des obstacles. Les 8 premieres sont les couleurs DOMINANTES lues dans les
# fichiers de materiaux (.mtl) du plan Sweet Home 3D ; les suivantes sont deduites de la
# categorie, faute de materiau fourni. Sans couleur, tous les blocs sortent gris et la
# scene est illisible.
COULEURS = [
    # (motif dans le nom, (r, g, b, a))
    ("brandt_ti612bt1", (0.14, 0.14, 0.14, 1.0)),   # plaque de cuisson (mtl)
    ("bluesky",         (0.94, 0.94, 0.94, 1.0)),   # micro-ondes (mtl)
    ("brandt_fp452",    (0.18, 0.18, 0.18, 1.0)),   # four (mtl)
    ("credence",        (0.85, 0.85, 0.85, 1.0)),   # credences (mtl)
    ("etagere_metal",   (0.68, 0.70, 0.72, 1.0)),   # etagere (mtl)
    ("support_robot",   (0.90, 0.76, 0.43, 1.0)),   # support du robot (mtl)
    ("mensola",         (1.00, 1.00, 1.00, 1.0)),   # tablette (mtl)
    ("schock_lithos",   (0.72, 0.73, 0.75, 1.0)),   # evier
    # deduites de la categorie
    ("wall",            (0.92, 0.90, 0.86, 1.0)),
    ("porte",           (0.76, 0.62, 0.42, 1.0)),
    ("poutre",          (0.55, 0.40, 0.25, 1.0)),
    ("frig",            (0.82, 0.84, 0.86, 1.0)),
    ("hotte",           (0.75, 0.77, 0.80, 1.0)),
    ("micro_onde",      (0.94, 0.94, 0.94, 1.0)),
    ("four",            (0.18, 0.18, 0.18, 1.0)),
    ("meuble",          (0.86, 0.73, 0.56, 1.0)),
    ("table_sous_robot", (0.90, 0.76, 0.43, 1.0)),
    ("plateau",         (0.90, 0.76, 0.43, 1.0)),
    ("pied",            (0.60, 0.60, 0.62, 1.0)),
    ("floor",           (0.78, 0.74, 0.68, 1.0)),
]
DEFAUT = (0.70, 0.70, 0.72, 1.0)


def couleur_de(nom):
    n = nom.lower()
    for motif, c in COULEURS:
        if motif in n:
            return list(c)
    return list(DEFAUT)



def lire_cubes(chemin):
    """Retourne [(groupe, nom, centre, dimensions, quaternion_wxyz)].

    Parcours direct : pour chaque `def Cube "nom"`, on isole le bloc { ... } qui suit
    en comptant les accolades, puis on lit ses attributs. Le groupe parent est le
    dernier `def "..."` sans type rencontre avant — il sert seulement a nommer.
    """
    texte = open(chemin, encoding="utf-8", errors="replace").read()
    groupes = [(m.start(), m.group(1))
               for m in re.finditer(r'def\s+"([^"]+)"', texte)]
    cubes = []
    for m in re.finditer(r'def\s+Cube\s+"([^"]+)"', texte):
        nom = m.group(1)
        i = texte.find("{", m.end())
        if i < 0:
            continue
        prof, j = 0, i
        while j < len(texte):
            if texte[j] == "{":
                prof += 1
            elif texte[j] == "}":
                prof -= 1
                if prof == 0:
                    break
            j += 1
        bloc = texte[i + 1:j]
        precedents = [g for pos, g in groupes if pos < m.start()]
        groupe = precedents[-1] if precedents else "obj"
        centre, dims, quat = _attributs(bloc)
        cubes.append((groupe, nom, centre, dims, quat))
    return cubes


def lire_meshes(chemin):
    """Retourne [(groupe, nom, centre, dimensions, quat)] : chaque Mesh devient sa BOITE
    ENGLOBANTE.

    Pourquoi : le plan 3D decrit certains elements (le support du robot, le sol) par des
    maillages et non par des primitives. Un maillage de collision est lourd a evaluer et
    MoveIt s'en accommode mal ; une boite englobante est conservatrice (elle ne peut que
    surestimer l'obstacle) et suffit pour empecher un contact.
    """
    texte = open(chemin, encoding="utf-8", errors="replace").read()
    groupes = [(m.start(), m.group(1)) for m in re.finditer(r'def\s+"([^"]+)"', texte)]
    out = []
    for m in re.finditer(r'def\s+Mesh\s+"([^"]+)"', texte):
        nom = m.group(1)
        i = texte.find("{", m.end())
        if i < 0:
            continue
        prof, j = 0, i
        while j < len(texte):
            if texte[j] == "{":
                prof += 1
            elif texte[j] == "}":
                prof -= 1
                if prof == 0:
                    break
            j += 1
        bloc = texte[i + 1:j]
        mp = re.search(r"point3f\[\]\s+points\s*=\s*\[(.*?)\]", bloc, re.S)
        if not mp:
            continue
        pts = [tuple(float(v) for v in t.split(","))
               for t in re.findall(r"\(([^)]+)\)", mp.group(1))]
        pts = [p for p in pts if len(p) == 3]
        if not pts:
            continue
        mins = [min(p[k] for p in pts) for k in range(3)]
        maxs = [max(p[k] for p in pts) for k in range(3)]
        centre_local = [(mins[k] + maxs[k]) / 2.0 for k in range(3)]
        dims = [max(maxs[k] - mins[k], MIN_EPAISSEUR) for k in range(3)]
        # transformation propre au mesh, si presente
        t = _vec(bloc, "xformOp:translate", [0.0, 0.0, 0.0])
        centre = [centre_local[k] + t[k] for k in range(3)]
        precedents = [g for pos, g in groupes if pos < m.start()]
        out.append((precedents[-1] if precedents else "mesh", nom, centre, dims,
                    _vec(bloc, "xformOp:orient", [1.0, 0.0, 0.0, 0.0])))
    return out


def _vec(bloc, cle, defaut):
    m = re.search(re.escape(cle) + r"\s*=\s*\(([^)]+)\)", bloc)
    if not m:
        return defaut
    return [float(x) for x in m.group(1).split(",")]


def _attributs(bloc):
    centre = _vec(bloc, "xformOp:translate", [0.0, 0.0, 0.0])
    dims = _vec(bloc, "xformOp:scale", [1.0, 1.0, 1.0])
    quat = _vec(bloc, "xformOp:orient", [1.0, 0.0, 0.0, 0.0])  # USD : (w, x, y, z)
    taille = 1.0
    m = re.search(r"double size\s*=\s*([\d.]+)", bloc)
    if m:
        taille = float(m.group(1))
    return centre, [d * taille for d in dims], quat


def pose_robot(chemin):
    """Extrait (translation, yaw) de la pose du robot dans la scene."""
    if chemin.endswith(".json"):
        import json

        d = json.load(open(chemin, encoding="utf-8"))
        t = d["translation_m"]
        w, x, y, z = d.get("rotation_quaternion_wxyz", [1, 0, 0, 0])
        return t, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    texte = open(chemin, encoding="utf-8", errors="replace").read()
    # On cherche la matrice appliquee au prim du robot (over "NIC").
    bloc = texte.split('over "NIC"')[-1]
    m = re.search(r"matrix4d[^=]*=\s*\(\s*(.+?)\s*\)\s*\n", bloc, re.S)
    if not m:
        sys.exit("pose du robot introuvable dans %s" % chemin)
    nombres = [float(x) for x in re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", m.group(1))]
    if len(nombres) < 16:
        sys.exit("matrice de pose incomplete")
    # USD : matrice ligne-majeure, translation sur la DERNIERE ligne.
    r00, r01 = nombres[0], nombres[1]
    t = nombres[12:15]
    return t, math.atan2(r01, r00)


def convertir(centre, quat, t_robot, yaw):
    """Repere cuisine -> repere base robot (inverse de la pose du robot)."""
    dx, dy, dz = (centre[0] - t_robot[0], centre[1] - t_robot[1], centre[2] - t_robot[2])
    c, s = math.cos(-yaw), math.sin(-yaw)
    x, y = c * dx - s * dy, s * dx + c * dy
    # yaw du cube dans le repere robot
    w, qx, qy, qz = quat
    yaw_cube = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return [x, y, dz], yaw_cube - yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("collision")
    ap.add_argument("--pose", required=True, help="USDA ou JSON donnant la pose du robot")
    ap.add_argument("--sortie", required=True)
    ap.add_argument("--rayon", type=float, default=2.5,
                    help="ne garde que les obstacles dont le centre est a moins de R (m)")
    ap.add_argument("--nom", default="cuisine")
    ap.add_argument("--exclure", action="append", default=[],
                    help="motif : n'importe PAS les obstacles dont le nom le contient "
                         "(ex. pour verifier un element via son maillage plutot que sa boite)")
    ap.add_argument("--rot-extra", type=float, default=0.0,
                    help="rotation SUPPLEMENTAIRE (deg) de toute la scene autour de Z, "
                         "appliquee apres le passage au repere robot. Sert quand le repere "
                         "du robot dans la scene ne coincide pas avec base_link.")
    ap.add_argument("--rogner", action="append", default=[],
                    help="prim:z_max — rabaisse le SOMMET d'un obstacle. Sert au support "
                         "qui PORTE le robot : leur contact permanent n'est pas une "
                         "collision, mais MoveIt le verrait comme telle et bloquerait tout.")
    ap.add_argument("--meshes", action="store_true",
                    help="importe aussi les Mesh, sous forme de boites englobantes")
    ap.add_argument("--extra", action="append", default=[],
                    help="prim:x,y,z,yaw_deg — transformation supplementaire a appliquer "
                         "a un prim (ex. le support deplace par le placement V2)")
    a = ap.parse_args()

    t_robot, yaw = pose_robot(a.pose)
    yaw += math.radians(a.rot_extra)
    if a.rot_extra:
        print("Rotation supplementaire de la scene : %+.0f deg" % a.rot_extra)
    print("Pose du robot dans la scene : t=(%.4f, %.4f, %.4f)  yaw=%.1f deg"
          % (*t_robot, math.degrees(yaw)))

    cubes = lire_cubes(a.collision)
    print("Boites lues : %d" % len(cubes))
    if a.meshes:
        maillages = lire_meshes(a.collision)
        print("Maillages -> boites englobantes : %d  (%s)"
              % (len(maillages), ", ".join(n for _, n, *_ in maillages)))
        cubes = cubes + maillages

    rognages = {}
    for r in a.rogner:
        prim, zmax = r.rsplit(":", 1)
        rognages[prim] = float(zmax)

    extras = {}
    for e in a.extra:
        prim, vals = e.split(":", 1)
        x, y, z, yd = (float(v) for v in vals.split(","))
        extras[prim] = (x, y, z, math.radians(yd))

    objets, ignores = [], 0
    noms_vus = {}
    for groupe, nom, centre, dims, quat in cubes:
        for prim, (ex, ey, ez, eyaw) in extras.items():
            if prim in groupe or prim in nom:
                c, sn = math.cos(eyaw), math.sin(eyaw)
                centre = [c * centre[0] - sn * centre[1] + ex,
                          sn * centre[0] + c * centre[1] + ey,
                          centre[2] + ez]
        pose, yaw_o = convertir(centre, quat, t_robot, yaw)
        if any(m.lower() in ("%s_%s" % (groupe, nom)).lower() for m in a.exclure):
            ignores += 1
            continue
        if math.hypot(pose[0], pose[1]) > a.rayon:
            ignores += 1
            continue
        base = "%s_%s" % (groupe, nom)
        base = re.sub(r"[^A-Za-z0-9_]", "_", base)[:48]
        noms_vus[base] = noms_vus.get(base, 0) + 1
        if noms_vus[base] > 1:
            base = "%s_%d" % (base, noms_vus[base])
        # Epaisseur minimale : le plan 3D contient des elements d'epaisseur nulle
        # (credence, plaques). Une boite plate est mal geree par le moteur de collision.
        dims = [max(abs(d), MIN_EPAISSEUR) for d in dims]
        for prim, zmax in rognages.items():
            if prim in base:
                sommet = pose[2] + dims[2] / 2.0
                if sommet > zmax:
                    nouvelle_h = max(MIN_EPAISSEUR, dims[2] - (sommet - zmax))
                    pose[2] = zmax - nouvelle_h / 2.0
                    dims[2] = nouvelle_h
                    print("  rogne %-34s sommet %.3f -> %.3f m" % (base[:34], sommet, zmax))
        o = {"name": base, "type": "box",
             "size": [round(d, 4) for d in dims],
             "pose": [round(p, 4) for p in pose]}
        o["color"] = couleur_de(base)
        if abs(yaw_o) > 1e-4:
            o["rpy"] = [0.0, 0.0, round(yaw_o, 5)]
        objets.append(o)

    print("Retenus : %d   (ignores hors rayon %.1f m : %d)" % (len(objets), a.rayon, ignores))
    doc = {
        "frame": "world",
        "objects": objets,
    }
    entete = (
        "# Environnement '%s' — importe du plan 3D de la cuisine (equipe Le NIC).\n"
        "# Source : %s\n"
        "# Pose du robot dans la scene : t=(%.4f, %.4f, %.4f), yaw=%.1f deg — les obstacles\n"
        "# sont exprimes dans le repere BASE ROBOT (world), comme le veut roby_environments.\n"
        "# Filtre : obstacles a moins de %.1f m de la base (les murs lointains sont inutiles\n"
        "# a la planification et alourdissent la scene).\n"
        "# GENERE par roby_usd_to_environment.py — ne pas editer a la main.\n"
        % (a.nom, os.path.basename(a.collision), *t_robot, math.degrees(yaw), a.rayon)
    )
    os.makedirs(os.path.dirname(os.path.abspath(a.sortie)), exist_ok=True)
    with open(a.sortie, "w", encoding="utf-8") as f:
        f.write(entete)
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
    print("-> %s" % a.sortie)
    return 0


if __name__ == "__main__":
    sys.exit(main())
