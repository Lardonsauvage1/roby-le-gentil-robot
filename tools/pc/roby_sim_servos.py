#!/usr/bin/env python3
"""Servos simules (verrou de tete + pince) pour `roby up --sim`.

Sur le vrai robot, RobySystem est l'unique abonne de /head_lock et /gripper. roby_sortie_nid
refuse de bouger sans abonne a /head_lock (deverrouillage non garanti) : en simulation il faut
donc quelqu'un pour tenir ce role. Ce noeud s'abonne, journalise l'etat, ne publie rien.

REFUSE le domaine 42 : la, un abonne de trop ferait croire que le verrou a un proprietaire.
"""

import os
import sys

import rclpy
from std_msgs.msg import Bool


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") == "42":
        print("roby_sim_servos : REFUS sur le domaine 42 (vrai robot).", file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node("roby_sim_servos")
    log = node.get_logger()
    node.create_subscription(
        Bool, "/head_lock", lambda m: log.info(f"verrou tete : {'VERROUILLE' if m.data else 'DEVERROUILLE'}"), 10
    )
    node.create_subscription(Bool, "/gripper", lambda m: log.info(f"pince : {'FERMEE' if m.data else 'OUVERTE'}"), 10)
    log.info("servos simules prets (/head_lock, /gripper)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
