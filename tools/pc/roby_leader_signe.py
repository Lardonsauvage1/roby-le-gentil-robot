#!/usr/bin/env python3
"""roby_leader_signe.py — inverse le signe d'un axe du bras guide, a chaud.

Regler les sens par essai-erreur est le seul moyen fiable : le signe correct depend
de l'orientation physique du servo sur le bras, qu'aucun calcul ne devine. Ce qui
compte, c'est que l'essai soit RAPIDE. Le noeud de teleoperation relit ce fichier
toutes les 0,5 s : on inverse, on regarde, on passe au suivant -- sans relancer quoi
que ce soit, et sans recouper le couple du guide.

  roby_leader_signe.py joint_4              inverse l'axe 4
  roby_leader_signe.py joint_2 joint_5      inverse plusieurs axes
  roby_leader_signe.py --etat               affiche les signes sans rien changer
"""
import os
import sys

import yaml

CHEMIN = os.path.expanduser(
    "~/ros2_ws/src/roby_control/config/leader_calibration.yaml")


def etat(d):
    for j in d["joints"]:
        print("  %-10s signe %+.0f   echelle %.4f" % (j["nom_urdf"], j["signe"],
                                                      j["echelle"]))


def main():
    args = [a for a in sys.argv[1:]]
    d = yaml.safe_load(open(CHEMIN))
    if not args or "--etat" in args:
        etat(d)
        return 0
    connus = {j["nom_urdf"] for j in d["joints"]}
    inconnus = [a for a in args if a not in connus]
    if inconnus:
        print("articulation inconnue : %s\nconnues : %s"
              % (", ".join(inconnus), ", ".join(sorted(connus))))
        return 1
    for j in d["joints"]:
        if j["nom_urdf"] in args:
            av = j["signe"]
            j["signe"] = -av
            j["notes"] = (j.get("notes", "") or "") + (
                " Signe %+.0f -> %+.0f (roby_leader_signe.py)." % (av, -av))
            print("  %-10s %+.0f -> %+.0f" % (j["nom_urdf"], av, -av))
    # Ecriture atomique : le noeud relit ce fichier en continu, il ne doit jamais
    # tomber sur un YAML tronque.
    tmp = CHEMIN + ".tmp"
    with open(tmp, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False, width=88)
    os.replace(tmp, CHEMIN)
    print("  -> pris en compte dans la seconde qui suit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
