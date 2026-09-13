"""Affichage direct : angles bruts du leader <-> angles convertis vers Roby (US-019).

Sert a REGLER signe / zero / echelle en observant : le fichier de calibration est relu
a chaque rafraichissement, donc on edite le YAML et on voit l'effet immediatement, sans
rien relancer.

Ne commande RIEN : lecture seule, aucun message vers le vrai bras.

    bash ~/roby_leader_calib.sh                 # necessite leader_node en marche
    bash ~/roby_leader_calib.sh --hz 5
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from roby_control.leader_mapping import DEFAULT_CALIB, charger

R2D = 180.0 / math.pi


class CalibLive(Node):
    def __init__(self, chemin, hz):
        super().__init__("leader_calib_live")
        self.chemin = chemin
        self.dernier = None
        self.create_subscription(JointState, "/leader/joint_states", self._on_js, 10)
        self.create_timer(1.0 / hz, self._afficher)
        self.get_logger().info(
            "Lecture de %s a chaque rafraichissement : editer le YAML met a jour "
            "l'affichage sans relancer." % chemin
        )

    def _on_js(self, msg):
        self.dernier = dict(zip(msg.name, msg.position))

    def _afficher(self):
        if self.dernier is None:
            print("\r(en attente de /leader/joint_states — leader_node tourne ?)",
                  end="", flush=True)
            return
        try:
            cal = charger(self.chemin)  # relu a chaud
        except Exception as e:  # noqa: BLE001
            print("\nCalibration illisible : %s" % e, flush=True)
            return

        lignes = ["", "%-18s %12s %12s %10s" % ("articulation", "brut(deg)",
                                                "converti(deg)", "etat")]
        for j in cal.joints:
            brut = self.dernier.get(j.nom_leader)
            if brut is None:
                lignes.append("%-18s %12s %12s %10s"
                              % (j.nom_leader, "absent", "-", "-"))
                continue
            q, clampe = j.convertir(brut)
            etat = "CLAMPE" if clampe else ("" if j.calibre else "non calibre")
            lignes.append("%-18s %12.1f %12.1f %10s"
                          % (j.nom_leader, brut * R2D, q * R2D, etat))
        non = cal.non_calibres()
        if non:
            lignes.append("[!] parametres non releves pour : %s — ces valeurs ne "
                          "doivent PAS servir a piloter." % ", ".join(non))
        print("\n".join(lignes), flush=True)


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    ap.add_argument("--hz", type=float, default=2.0)
    a, ros_args = ap.parse_known_args(args if args is not None else sys.argv[1:])
    if not os.path.exists(a.calib):
        sys.exit("calibration introuvable : %s" % a.calib)
    rclpy.init(args=ros_args)
    node = CalibLive(a.calib, max(0.2, a.hz))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
