#!/usr/bin/env python3
"""roby_leader_zero.py — capture les ZEROS du bras guide sur sa pose actuelle.

Le probleme qu'il resout : les zeros d'US-017 ont ete CALCULES (milieu de la course
mesuree), jamais releves en placant le guide dans une pose reelle. Resultat, le bras
simule n'est pas aligne sur le vrai : on tient le guide dans une pose, et Roby en
affiche une autre.

Principe : on met le guide dans une pose choisie, on dit a quelle pose de Roby elle
doit correspondre, et on capture.

    zero      <- position brute lue MAINTENANT sur chaque servo
    zero_urdf <- angle Roby que cette pose doit produire

La conversion etant  q_roby = zero_urdf + signe * echelle * ecart_au_zero,  la pose
capturee donne exactement `zero_urdf`. L'alignement est donc EXACT en ce point, et
l'echelle gere le reste de la course.

  roby_leader_zero.py                 # Roby au ZERO de ses articulations (defaut)
  roby_leader_zero.py --roby centre   # Roby au CENTRE de sa course
  roby_leader_zero.py --montrer       # ne rien ecrire, juste montrer ce qui serait ecrit

Ne touche PAS aux signes ni aux echelles : seulement zero et zero_urdf.
"""
import argparse
import math
import os
import sys
import time

import yaml

CHEMIN = os.path.expanduser(
    "~/ros2_ws/src/roby_control/config/leader_calibration.yaml")


def lire_guide(duree=3.0):
    """-> {nom_leader: rad}. Lecture de /leader/joint_states."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    recu = {}

    class L(Node):
        def __init__(self):
            super().__init__("capture_zero")
            self.create_subscription(JointState, "/leader/joint_states",
                                     self.cb, 10)

        def cb(self, m):
            for n, p in zip(m.name, m.position):
                recu[n] = p

    rclpy.init()
    n = L()
    t = time.time()
    while time.time() - t < duree and len(recu) < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()
    return recu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roby", choices=["zero", "centre"], default="zero",
                    help="pose de Roby qui doit correspondre a la pose actuelle du guide")
    ap.add_argument("--montrer", action="store_true",
                    help="afficher sans ecrire")
    a = ap.parse_args()

    d = yaml.safe_load(open(CHEMIN))
    pos = lire_guide()
    if not pos:
        print("aucune donnee sur /leader/joint_states — leader_node tourne-t-il ?")
        return 1

    print("  axe              zero actuel -> nouveau      angle Roby vise")
    modifs = 0
    for j in d["joints"]:
        nom = j["nom_leader"]
        if nom not in pos:
            continue
        cible = 0.0 if a.roby == "zero" else (j["urdf_min"] + j["urdf_max"]) / 2.0
        cible = max(j["urdf_min"], min(j["urdf_max"], cible))
        print("  %-18s %6.1f -> %6.1f deg        %+6.1f deg"
              % (nom, math.degrees(j["zero"]), math.degrees(pos[nom]),
                 math.degrees(cible)))
        if not a.montrer:
            j["zero"] = float(pos[nom])
            j["zero_urdf"] = float(cible)
            j["notes"] = (j.get("notes", "") or "") + (
                " Zero RELEVE sur la pose reelle du guide (roby_leader_zero.py), "
                "correspondance -> %.1f deg cote Roby." % math.degrees(cible))
        modifs += 1

    if a.montrer:
        print("\n  (--montrer : rien n'a ete ecrit)")
        return 0
    tmp = CHEMIN + ".tmp"
    with open(tmp, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False, width=88)
    os.replace(tmp, CHEMIN)
    print("\n  %d axes recales -> pris en compte dans la seconde qui suit" % modifs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
