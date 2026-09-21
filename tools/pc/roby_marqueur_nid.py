#!/usr/bin/env python3
"""Publie un marqueur RViz a la position du NID, dans le repere de la base du robot.

Sert a verifier a l'oeil l'alignement entre la simulation et la realite : le nid est
le seul point dont la position est connue des DEUX cotes — par la cinematique du robot
(pose articulaire de docking) et par le plan 3D de la cuisine.

C'est un marqueur VISUEL, pas un obstacle : l'ajouter a la scene de planification
bloquerait le bras chaque fois qu'il veut aller se docker.

Position (repere base robot) — RELEVEE EN SIMULATION le 2026-09-20 :

    pose articulaire   [1.56020, 0.93500, 0.58883, -0.00249, 0.04700] rad
                       [89.393, 53.572, 33.737, -0.143, 2.693] deg
    changeur           (+4.495, +423.298, -56.001) mm
    centre du poignet  (+4.479, +422.714, +36.319) mm
    bout de l'outil    (+4.507, +423.294, -156.001) mm

Comment elle a ete obtenue : NM a amene le bras simule jusqu'a ce que le modele SOLIDE
coincide avec le berceau, apres avoir masque les robots translucides de MoveIt
(Trajectory et Planning Request) qui ne montrent ni l'un ni l'autre l'etat reel.

⚠️ C'est un reglage A L'OEIL sur un modele 3D, pas une mesure physique. Il remplace
l'ancienne pose (perimee, confirme par NM) mais il ne vaut que ce que vaut la geometrie
corrigee le 2026-09-20. La mesure qui ferait foi reste : tete posee dans le berceau,
moteurs coupes, lecture de /joint_states sur le vrai robot.

Historique des valeurs essayees pour X ce jour-la : 5.5 -> 4.5 -> 6.0 -> 8.0 -> 18.0,
puis abandonnees au profit du relevé ci-dessus. Les vecteurs du 2026-09-06 (maillage ->
nid = (+5, +323.50, +125) mm, maillage -> base = (-2, -171.824, +203) mm) donnaient
(+7, +495.324, -78) mm ; ils sont desormais caduques.

    bash ~/roby_marqueur_nid.sh
"""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

NID = (0.004495, 0.423298, -0.056001)   # m — releve en simulation (voir en-tete)
# Pose articulaire du nid ainsi relevee — a confronter au vrai robot avant de
# l'ecrire dans src/roby_hardware/config/initial_positions.yaml.
Q_NID = (1.56020, 0.93500, 0.58883, -0.00249, 0.04700)   # rad
TCP_FK = (0.004495, 0.423298, -0.056001)  # meme point par la cinematique : confondu


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

    def publier(self):
        a = MarkerArray()
        # Le socle n'est plus dessine ici : il est devenu le corps fixe "socle" de l'URDF
        # et s'affiche via le RobotModel. En garder une copie a l'echelle 1.0 l'aurait
        # rendu 1000x trop grand depuis que le maillage livre est en millimetres.
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
