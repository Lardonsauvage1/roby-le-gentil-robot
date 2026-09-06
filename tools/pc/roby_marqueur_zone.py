#!/usr/bin/env python3
"""Affiche la ZONE DE TRAVAIL (generation de datasets) dans RViz.

Publie sur le meme topic que le marqueur du nid (`/roby/reperes`), avec un espace de
noms distinct : un seul affichage a ajouter dans RViz pour voir les deux.

Les bornes sont des PARAMETRES : on les ajuste et on relance, sans toucher au code.

    bash ~/roby_marqueur_zone.sh
    bash ~/roby_marqueur_zone.sh -p x_min:=0.25 -p y_max:=0.05
"""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MarqueurZone(Node):
    def __init__(self):
        super().__init__("marqueur_zone")
        # Zone deduite des obstacles de la cuisine (2026-09-06) :
        #   X : bord de la structure sous le robot -> aplomb du refrigerateur
        #   Y : bord du refrigerateur -> bord de la plaque de cuisson
        for n, v in (("x_min", 0.19), ("x_max", 0.89),
                     ("y_min", -0.56), ("y_max", 0.10),
                     ("z", -0.007), ("epaisseur", 0.02),
                     # Hauteur de PRISE : la zone de tirage se place au-dessus de la
                     # surface, a la meme distance que dans l'ancienne scene (26,2 cm
                     # mesures entre le plan de travail de l'atelier et la hauteur du
                     # point de depose D).
                     ("hauteur_prise", 0.2621)):
            self.declare_parameter(n, v)
        g = lambda n: float(self.get_parameter(n).value)
        self.b = [g("x_min"), g("x_max"), g("y_min"), g("y_max"), g("z"), g("epaisseur")]
        self.h = g("hauteur_prise")
        self.pub = self.create_publisher(MarkerArray, "/roby/reperes", 1)
        self.create_timer(1.0, self.publier)
        x0, x1, y0, y1, z, e = self.b
        self.get_logger().info(
            "Zone : X %.3f..%.3f  Y %.3f..%.3f  (%.0f x %.0f cm) | surface z=%.3f | "
            "prise z=%.3f (%.1f cm au-dessus)"
            % (x0, x1, y0, y1, (x1-x0)*100, (y1-y0)*100, z, z + self.h, self.h*100))

    def publier(self):
        x0, x1, y0, y1, z, e = self.b
        a = MarkerArray()
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "zone", 0, Marker.CUBE, Marker.ADD
        m.pose.position = Point(x=(x0+x1)/2, y=(y0+y1)/2, z=z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (x1-x0), (y1-y0), e
        m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.9, 0.9, 0.35)
        a.markers.append(m)

        # contour, pour voir les limites meme vu de dessus
        c = Marker()
        c.header = m.header
        c.ns, c.id, c.type, c.action = "zone", 1, Marker.LINE_STRIP, Marker.ADD
        c.pose.orientation.w = 1.0
        c.scale.x = 0.008
        c.color.r, c.color.g, c.color.b, c.color.a = (0.0, 1.0, 1.0, 1.0)
        for x, y in ((x0,y0), (x1,y0), (x1,y1), (x0,y1), (x0,y0)):
            c.points.append(Point(x=x, y=y, z=z + e/2 + 0.001))
        a.markers.append(c)

        t = Marker()
        t.header = m.header
        t.ns, t.id, t.type, t.action = "zone", 2, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position = Point(x=(x0+x1)/2, y=(y0+y1)/2, z=z + 0.08)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.05
        t.color.r, t.color.g, t.color.b, t.color.a = (0.0, 1.0, 1.0, 1.0)
        t.text = "surface  %.0fx%.0f cm" % ((x1-x0)*100, (y1-y0)*100)
        a.markers.append(t)

        # --- plan de PRISE, a la meme distance de la surface que dans l'ancienne scene ---
        zp = z + self.h
        pr = Marker()
        pr.header = m.header
        pr.ns, pr.id, pr.type, pr.action = "zone", 3, Marker.CUBE, Marker.ADD
        pr.pose.position = Point(x=(x0+x1)/2, y=(y0+y1)/2, z=zp)
        pr.pose.orientation.w = 1.0
        pr.scale.x, pr.scale.y, pr.scale.z = (x1-x0), (y1-y0), 0.004
        pr.color.r, pr.color.g, pr.color.b, pr.color.a = (1.0, 0.85, 0.1, 0.45)
        a.markers.append(pr)

        tp = Marker()
        tp.header = m.header
        tp.ns, tp.id, tp.type, tp.action = "zone", 4, Marker.TEXT_VIEW_FACING, Marker.ADD
        tp.pose.position = Point(x=(x0+x1)/2, y=(y0+y1)/2, z=zp + 0.06)
        tp.pose.orientation.w = 1.0
        tp.scale.z = 0.05
        tp.color.r, tp.color.g, tp.color.b, tp.color.a = (1.0, 0.85, 0.1, 1.0)
        tp.text = "prise  +%.1f cm" % (self.h * 100)
        a.markers.append(tp)

        # trait vertical entre les deux plans, pour materialiser la distance
        li = Marker()
        li.header = m.header
        li.ns, li.id, li.type, li.action = "zone", 5, Marker.LINE_LIST, Marker.ADD
        li.pose.orientation.w = 1.0
        li.scale.x = 0.006
        li.color.r, li.color.g, li.color.b, li.color.a = (1.0, 1.0, 1.0, 0.9)
        for x, y in ((x0,y0), (x1,y0), (x1,y1), (x0,y1)):
            li.points.append(Point(x=x, y=y, z=z))
            li.points.append(Point(x=x, y=y, z=zp))
        a.markers.append(li)
        self.pub.publish(a)


def main(args=None):
    rclpy.init(args=args)
    n = MarqueurZone()
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
