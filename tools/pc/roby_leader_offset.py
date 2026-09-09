#!/usr/bin/env python3
"""roby_leader_offset.py — decale la correspondance d'un axe de N degres.

Agit sur `zero_urdf` : l'angle Roby obtenu quand le guide est a son zero. C'est le
reglage a utiliser quand les deux bras suivent bien mais ne POINTENT pas dans la meme
direction -- typiquement une base montee a 180 deg de l'autre.

  roby_leader_offset.py joint_1 180          decale zero_urdf de +180 deg
  roby_leader_offset.py --guide joint_2 10   decale le ZERO DU GUIDE de +10 deg
  roby_leader_offset.py --etat               affiche les correspondances actuelles

Deux reglages a ne pas confondre :

  (defaut)   agit sur `zero_urdf` -- l'angle ROBY au repos. Le simule bouge, le guide
             ne bouge pas. A utiliser quand le simule ne POINTE pas dans la bonne
             direction.
  --guide    agit sur `zero` -- la position LEADER de reference. C'est la pose ou le
             guide se gare au recentrage, et l'origine de la conversion. A utiliser
             quand les deux bras suivent bien mais restent decales d'un offset
             constant.

Ne touche ni au signe ni a l'echelle.
"""
import math
import os
import sys

import yaml

CHEMIN = os.path.expanduser(
    "~/ros2_ws/src/roby_control/config/leader_calibration.yaml")


def etat(d):
    for j in d["joints"]:
        print("  %-10s zero_urdf %+7.1f deg   butees [%+7.1f, %+7.1f]"
              % (j["nom_urdf"], math.degrees(j["zero_urdf"]),
                 math.degrees(j["urdf_min"]), math.degrees(j["urdf_max"])))


def main():
    args = sys.argv[1:]
    d = yaml.safe_load(open(CHEMIN))
    if not args or args[0] == "--etat":
        etat(d)
        return 0
    guide = "--guide" in args
    args = [a for a in args if a != "--guide"]
    if len(args) != 2:
        print(__doc__)
        return 1
    noms, deg = args[0].split(","), float(args[1])
    touches = 0
    for j in d["joints"]:
        if j["nom_urdf"] not in noms:
            continue
        touches += 1
        if guide:
            av = j["zero"]
            ap = (av + math.radians(deg)) % (2 * math.pi)
            j["zero"] = float(ap)
            j["notes"] = (j.get("notes", "") or "") + (
                " zero guide %+.1f -> %+.1f deg (decalage de %+.0f deg, "
                "roby_leader_offset.py --guide)." % (math.degrees(av),
                                                     math.degrees(ap), deg))
            dep = -j["signe"] * j["echelle"] * math.radians(deg)
            print("  %-10s zero GUIDE %+7.1f -> %+7.1f deg   "
                  "(le simule se decale de %+.1f deg a pose du guide inchangee)"
                  % (j["nom_urdf"], math.degrees(av), math.degrees(ap),
                     math.degrees(dep)))
            continue
        av = j["zero_urdf"]
        ap = av + math.radians(deg)
        # On ramene dans [-pi, pi] : un zero_urdf hors de cette fenetre n'a pas de sens
        # et sortirait des butees des le repos.
        ap = (ap + math.pi) % (2 * math.pi) - math.pi
        j["zero_urdf"] = float(ap)
        j["notes"] = (j.get("notes", "") or "") + (
            " zero_urdf %+.1f -> %+.1f deg (decalage de %+.0f deg, "
            "roby_leader_offset.py)." % (math.degrees(av), math.degrees(ap), deg))
        print("  %-10s zero_urdf %+7.1f -> %+7.1f deg" % (nom, math.degrees(av),
                                                          math.degrees(ap)))
        marge_bas = math.degrees(ap - j["urdf_min"])
        marge_haut = math.degrees(j["urdf_max"] - ap)
        print("     marge restante : %+.1f deg d'un cote, %+.1f de l'autre"
              % (marge_bas, marge_haut))
        if min(marge_bas, marge_haut) < 30:
            print("     /!\\ la nouvelle position de repos est TRES pres d'une butee :"
                  "\n         de ce cote-la, le suivi sera clampe presque tout de suite.")
    if not touches:
        print("articulation inconnue : %s" % ", ".join(noms))
        return 1
    tmp = CHEMIN + ".tmp"
    with open(tmp, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False, width=88)
    os.replace(tmp, CHEMIN)
    print("  -> pris en compte dans la seconde qui suit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
