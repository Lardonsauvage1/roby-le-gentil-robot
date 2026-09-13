#!/usr/bin/env python3
"""Publie un marqueur RViz a la position du NID, dans le repere de la base du robot.

Sert a verifier a l'oeil l'alignement entre la simulation et la realite : le nid est
le seul point dont la position est connue des DEUX cotes — par la cinematique du robot
(pose articulaire de docking) et par le plan 3D de la cuisine.

C'est un marqueur VISUEL, pas un obstacle : l'ajouter a la scene de planification
bloquerait le bras chaque fois qu'il veut aller se docker.

Position (repere base robot), deduite des vecteurs mesures par Sam le 2026-09-06 :
    origine du maillage -> base robot : (-2, -171.824, +203) mm
    origine du maillage -> nid        : (+5, +323.50, +125) mm
    donc base robot -> nid            : (+7, +495.324, -78) mm

Controle croise : la cinematique directe a la pose de docking donne
(+4.0, +490.2, -18.1) mm, soit 3 mm en X et 5 mm en Y d'ecart — l'alignement est bon.
Les 60 mm d'ecart en Z sont exactement l'offset TCP de notre `fkT` (le vecteur de Sam
vise le COUPLEUR, notre calcul donne le TCP, 6 cm plus loin).

    bash ~/roby_marqueur_nid.sh
"""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

NID = (0.007, 0.495324, -0.078)      # m, repere base robot
TCP_FK = (0.004, 0.490200, -0.018)   # m, meme point vu par notre cinematique


class MarqueurNid(Node):
    def __init__(self):
        super().__init__("marqueur_nid")
        self.pub = self.create_publisher(MarkerArray, "/roby/reperes", 1)
        self.create_timer(1.0, self.publier)
        self.get_logger().info(
            "Marqueur du nid publie sur /roby/reperes (repere base_link). "
            "Dans RViz : Add -> MarkerArray -> topic /roby/reperes")

    def _m(self, i, pos, couleur, taille, texte=None):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "nid", i
        m.action = Marker.ADD
        m.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        m.pose.orientation.w = 1.0
        if texte is None:
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = taille
        else:
            m.type = Marker.TEXT_VIEW_FACING
            m.scale.z = taille
            m.text = texte
            m.pose.position.z += 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = couleur
        return m

    def _mesh(self, i):
        """Le support REEL, avec le nid dedans — pour verifier a l'oeil que le nid
        tombe au bon endroit dans la piece."""
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "nid", i
        m.action = Marker.ADD
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = "package://roby_environments/meshes/support_robot.stl"
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = (0.55, 0.55, 0.60, 0.45)
        return m

    def publier(self):
        a = MarkerArray()
        a.markers.append(self._mesh(4))
        # Vert : le nid d'apres les mesures physiques (reference)
        a.markers.append(self._m(0, NID, (0.1, 0.9, 0.1, 0.9), 0.03))
        a.markers.append(self._m(1, NID, (0.1, 0.9, 0.1, 1.0), 0.03, "nid (mesure)"))
        # Orange : le meme point vu par la cinematique — l'ecart se voit d'un coup d'oeil
        a.markers.append(self._m(2, TCP_FK, (1.0, 0.6, 0.0, 0.7), 0.025))
        a.markers.append(self._m(3, TCP_FK, (1.0, 0.6, 0.0, 1.0), 0.025, "TCP cinematique"))
        self.pub.publish(a)


def main(args=None):
    rclpy.init(args=args)
    n = MarqueurNid()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
