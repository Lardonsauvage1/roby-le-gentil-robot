#!/usr/bin/env python3
"""Squelette du bras : la chaine cinematique SEULE, sans aucun maillage.

Pourquoi : les STL sont ancres a `xyz="0 0 0"` dans le repere de leur corps, donc ils
suivent leur articulation. Quand une origine d'articulation bouge, le maillage bouge avec
elle et on ne voit plus que le desordre visuel -- pas la geometrie. Ici on ne dessine que
ce que le code utilise vraiment : les origines d'articulations, les segments qui les
relient, et l'axe de rotation de chacune.

La chaine est lue dans l'URDF par roby_cinematique (ADR-005) : ce qui est affiche est
exactement ce que calculent l'IK, la garde et la teleop.

    source ~/roby_env.sh          # ou ROBY_SIM=1 pour la simulation
    python3 ~/roby_squelette.py   # publie /roby/squelette (MarkerArray)
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))  # voisins de CE fichier

import roby_cinematique as cin  # noqa: E402

REPERE = "world"
J = [f"joint_{i}" for i in range(1, 6)]

# Une couleur par articulation, pour les reconnaitre sans lire les etiquettes.
COULEURS = [(0.95, 0.26, 0.21), (0.99, 0.60, 0.15), (0.95, 0.85, 0.20),
            (0.30, 0.79, 0.35), (0.25, 0.55, 0.96)]


def _socle_urdf():
    """Placement du socle, LU DANS L'URDF : (fichier, echelle, xyz, rpy).

    Le maillage livre est en millimetres et son origine est au bas du socle ; c'est
    l'URDF qui porte l'echelle et le recalage. On ne les recopie pas ici (ADR-005).
    """
    import xml.etree.ElementTree as ET
    for lk in ET.parse(cin.CHEMIN_URDF).getroot().iter("link"):
        if lk.get("name") != "socle":
            continue
        v = lk.find("visual")
        m = v.find("geometry/mesh") if v is not None else None
        if m is None:
            break
        o = v.find("origin")
        lire = lambda a, d: np.array(
            [float(x) for x in ((o.get(a, d).split()) if o is not None else d.split())])
        ech = np.array([float(x) for x in m.get("scale", "1 1 1").split()])
        return m.get("filename"), ech, lire("xyz", "0 0 0"), lire("rpy", "0 0 0")
    return None


def _offset_tcp():
    """Decalage link_gripper -> tcp, LU DANS L'URDF (jamais recopie, cf. ADR-005)."""
    import xml.etree.ElementTree as ET
    for j in ET.parse(cin.CHEMIN_URDF).getroot().iter("joint"):
        enfant = j.find("child")
        if enfant is not None and enfant.get("link") == "tcp":
            o = j.find("origin")
            return np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
    raise ValueError(f"aucune articulation vers 'tcp' dans {cin.CHEMIN_URDF}")


def _quat(rpy):
    """rpy fixes XYZ -> quaternion (w, x, y, z), convention URDF."""
    r, p, y = [float(v) / 2 for v in rpy]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def rgba(c, a=1.0):
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(a))


class Squelette(Node):
    def __init__(self):
        super().__init__("roby_squelette")
        self.declare_parameter("epaisseur", 0.012)      # m, diametre des segments
        self.declare_parameter("longueur_axe", 0.10)    # m, longueur des fleches d'axe
        self.declare_parameter("etiquettes", True)
        # Le socle sur lequel le bras est pose : corps fixe "socle" de l'URDF. Son
        # maillage, son echelle et son recalage sont lus la-bas, jamais recopies ici.
        self.declare_parameter("socle", True)
        self.q = None
        self.socle = _socle_urdf()
        self.declare_parameter("mesure_point", "tcp")   # "tcp" ou "link_gripper"
        self.pub = self.create_publisher(MarkerArray, "/roby/squelette", 1)
        # Topic separe : la mesure peut s'afficher dans la vue cuisine sans y amener
        # tout le squelette.
        self.pub_mes = self.create_publisher(MarkerArray, "/roby/mesures", 1)
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.create_timer(0.1, self._publier)
        self.get_logger().info(
            f"Squelette du bras, chaine lue dans {cin.CHEMIN_URDF} — "
            f"{len(cin.CHAINE)} segments, repere '{REPERE}'. Aucun maillage.")

    def _js(self, m):
        par = dict(zip(m.name, m.position))
        if all(j in par for j in J):
            self.q = [float(par[j]) for j in J]

    # ------------------------------------------------------------------ geometrie
    def _points(self):
        """Origine de chaque articulation, dans l'ordre de la chaine, + l'outil."""
        pts, axes, noms = [], [], []
        k = 0
        for i, seg in enumerate(cin.CHAINE):
            T = cin._parcours(self.q, i + 1)
            pts.append(T[:3, 3])
            noms.append(seg["nom"])
            if seg["fixe"]:
                axes.append(None)
            else:
                # l'axe est declare dans le repere du corps APRES l'origine fixe
                Tp = cin._parcours(self.q, i)
                R = Tp[:3, :3] @ cin._rpy(*seg["rpy"])
                axes.append(R @ np.asarray(seg["axe"], float))
                k += 1
        return pts, axes, noms

    # ------------------------------------------------------------------ marqueurs
    def _publier(self):
        if self.q is None:
            return
        ep = self.get_parameter("epaisseur").value
        la = self.get_parameter("longueur_axe").value
        etiq = self.get_parameter("etiquettes").value
        pts, axes, noms = self._points()
        ma = MarkerArray()
        n = 0

        def neuf(type_, echelle, couleur):
            nonlocal n
            m = Marker()
            m.header.frame_id = REPERE
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "squelette"
            m.id = n
            n += 1
            m.type = type_
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=float(echelle[0]), y=float(echelle[1]), z=float(echelle[2]))
            m.color = couleur
            return m

        # 1. les os : un segment de la base vers joint_1, puis d'origine en origine
        os_ = neuf(Marker.LINE_STRIP, (ep, 0, 0), rgba((0.55, 0.57, 0.60), 0.95))
        os_.points = [Point(x=0.0, y=0.0, z=0.0)] + [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        ma.markers.append(os_)

        # 2. une bille sur chaque origine d'articulation, couleur par axe
        ia = 0
        for p, ax, nom in zip(pts, axes, noms):
            fixe = ax is None
            c = rgba((0.45, 0.45, 0.45) if fixe else COULEURS[ia % len(COULEURS)])
            d = ep * (1.6 if fixe else 2.4)
            b = neuf(Marker.SPHERE, (d, d, d), c)
            b.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            ma.markers.append(b)

            # 3. l'axe de rotation, traverse la bille pour qu'on voie son orientation
            if not fixe:
                u = ax / (np.linalg.norm(ax) or 1.0)
                fl = neuf(Marker.ARROW, (ep * 0.45, ep * 0.9, ep * 1.2), c)
                fl.points = [
                    Point(x=float(p[0] - u[0] * la / 2), y=float(p[1] - u[1] * la / 2),
                          z=float(p[2] - u[2] * la / 2)),
                    Point(x=float(p[0] + u[0] * la / 2), y=float(p[1] + u[1] * la / 2),
                          z=float(p[2] + u[2] * la / 2))]
                ma.markers.append(fl)
                ia += 1

            if etiq:
                t = neuf(Marker.TEXT_VIEW_FACING, (0, 0, ep * 2.2), rgba((0.95, 0.95, 0.95)))
                t.text = nom
                t.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2] + d))
                ma.markers.append(t)

        # 4. le point que protege la garde (link_gripper) : un cube, pour le distinguer
        po = cin.fk_pos(self.q)
        cube = neuf(Marker.CUBE, (ep * 2, ep * 2, ep * 2), rgba((1.0, 0.0, 0.6)))
        cube.pose.position = Point(x=float(po[0]), y=float(po[1]), z=float(po[2]))
        ma.markers.append(cube)

        # 5. le centre du poignet : le point que commande la teleop cartesienne
        pp = cin.fk_poignet(self.q)
        cyl = neuf(Marker.SPHERE, (ep * 2.2, ep * 2.2, ep * 2.2), rgba((0.0, 0.9, 0.9), 0.8))
        cyl.pose.position = Point(x=float(pp[0]), y=float(pp[1]), z=float(pp[2]))
        ma.markers.append(cyl)

        # 6. le socle : seul solide affiche, translucide, pour ne pas masquer la chaine
        if self.get_parameter("socle").value and self.socle is not None:
            fichier, ech, xyz, rpy = self.socle
            s = neuf(Marker.MESH_RESOURCE, ech, rgba((0.35, 0.40, 0.45), 0.35))
            s.mesh_resource = fichier
            s.mesh_use_embedded_materials = False
            s.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
            qw, qx, qy, qz = _quat(rpy)
            s.pose.orientation.w, s.pose.orientation.x = float(qw), float(qx)
            s.pose.orientation.y, s.pose.orientation.z = float(qy), float(qz)
            ma.markers.append(s)

        self.pub.publish(ma)
        self._publier_mesure(ep)

    # ------------------------------------------------------------------ mesure base -> outil
    def _point_mesure(self):
        """Point dont on mesure l'ecart a la base, et son nom."""
        T = cin.fkT(self.q)
        if self.get_parameter("mesure_point").value == "link_gripper":
            return T[:3, 3], "link_gripper"
        return T[:3, 3] + T[:3, :3] @ _offset_tcp(), "tcp"

    def _mesures(self):
        """Les couples de points a coter : (nom, point de depart, point d'arrivee).

        Tout vient de la cinematique lue dans l'URDF — aucune distance n'est ecrite ici.
        """
        p_outil, nom_outil = self._point_mesure()
        poignet = cin.fk_poignet(self.q)          # origine de joint_5 = axe du poignet
        pince = cin.fk_pos(self.q)                # link_gripper = interface du changeur
        j1 = cin._parcours(self.q, 1)[:3, 3]      # origine de joint_1, posee sur le socle
        return [
            (f"base -> {nom_outil}", np.zeros(3), p_outil),
            ("axe du poignet -> changeur", poignet, pince),
            ("joint_1 -> changeur (nid)", j1, pince),
        ]

    def _publier_mesure(self, ep):
        """Chaque mesure : le segment direct, sa decomposition x/y/z, et les cotes."""
        ma = MarkerArray()
        n = [0]

        def neuf(type_, echelle, couleur, ns):
            m = Marker()
            m.header.frame_id = REPERE
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = n[0]
            n[0] += 1
            m.type = type_
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=float(echelle[0]), y=float(echelle[1]), z=float(echelle[2]))
            m.color = couleur
            return m

        def P(v):
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        # Triede des axes a la base : lever l'ambiguite "gauche / droite", qui n'est
        # pas une grandeur du modele mais une direction a l'ecran.
        for u, coul, lettre in ((np.array([1., 0, 0]), (1.0, 0.25, 0.25), "x  avant"),
                                (np.array([0, 1., 0]), (0.25, 1.0, 0.35), "y  gauche"),
                                (np.array([0, 0, 1.]), (0.35, 0.55, 1.0), "z  haut")):
            fl = neuf(Marker.ARROW, (ep * 0.5, ep * 1.1, ep * 1.6), rgba(coul), "axes")
            fl.points = [P(np.zeros(3)), P(u * 0.15)]
            ma.markers.append(fl)
            t = neuf(Marker.TEXT_VIEW_FACING, (0, 0, ep * 1.7), rgba(coul), "axes")
            t.text = lettre
            t.pose.position = P(u * 0.17)
            ma.markers.append(t)

        for titre, a, b in self._mesures():
            ns = "mesure"
            # segment direct
            direct = neuf(Marker.LINE_STRIP, (ep * 0.8, 0, 0), rgba((1.0, 1.0, 1.0), 0.95), ns)
            direct.points = [P(a), P(b)]
            ma.markers.append(direct)

            # decomposition : x, puis y, puis z, chacun dans sa couleur
            d = b - a
            coude_x = a + np.array([d[0], 0.0, 0.0])
            coude_y = coude_x + np.array([0.0, d[1], 0.0])
            for p0, p1, coul, lettre in ((a, coude_x, (1.0, 0.25, 0.25), "x"),
                                         (coude_x, coude_y, (0.25, 1.0, 0.35), "y"),
                                         (coude_y, b, (0.35, 0.55, 1.0), "z")):
                seg = neuf(Marker.LINE_STRIP, (ep * 0.55, 0, 0), rgba(coul, 0.9), ns)
                seg.points = [P(p0), P(p1)]
                ma.markers.append(seg)
                t = neuf(Marker.TEXT_VIEW_FACING, (0, 0, ep * 1.7), rgba(coul), ns)
                t.text = f"{lettre} = {np.linalg.norm(p1 - p0) * 1000:.1f}"
                t.pose.position = P((p0 + p1) / 2 + np.array([0, 0, ep * 1.3]))
                ma.markers.append(t)

            t = neuf(Marker.TEXT_VIEW_FACING, (0, 0, ep * 1.9), rgba((1.0, 1.0, 1.0)), ns)
            t.text = f"{titre} : {np.linalg.norm(d) * 1000:.1f} mm"
            t.pose.position = P((a + b) / 2 + np.array([0, 0, ep * 3.5]))
            ma.markers.append(t)

        self.pub_mes.publish(ma)


def main():
    rclpy.init()
    n = Squelette()
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
