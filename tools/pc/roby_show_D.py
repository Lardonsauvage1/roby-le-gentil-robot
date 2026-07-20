#!/usr/bin/env python3
"""Affiche le point de depose D (roby_oracle) dans RViz, SANS bouger le bras.

Publie une MarkerArray (latched) sur /oracle_debug, repere 'world' :
  - sphere verte = D_XYZ (point de prise/depose enregistre)
  - 3 fleches R/V/B = repere d'orientation de prise R_GRASP (x/y/z de l'outil)
  - trait gris vertical = descente ligne droite (z_lift -> z_pick)
  - rectangle blanc = contour physique de la table
  - rectangle jaune = fenetre de tirage aleatoire autour de D
Aucun /joint_states, aucun /arm_controller : purement visuel.
"""
import os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

sys.path.insert(0, os.path.expanduser("~"))
import roby_oracle as o

FRAME = "world"
Dx, Dy, Dz = o.D_XYZ
Rg = o.R_GRASP                      # orientation de prise (3x3)
z_lift = Dz + o.LIFT


def _m(mid, mtype, r, g, b, a=1.0, sx=0.01, sy=0.01, sz=0.01):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = "oracle_D"; m.id = mid; m.type = mtype; m.action = Marker.ADD
    m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
    m.pose.orientation.w = 1.0
    return m


def _pt(x, y, z):
    p = Point(); p.x, p.y, p.z = float(x), float(y), float(z); return p


def build():
    arr = MarkerArray()

    # 1) sphere D (3 cm ~ taille cone)
    s = _m(0, Marker.SPHERE, 0.1, 0.9, 0.1, 0.9, 0.03, 0.03, 0.03)
    s.pose.position = _pt(Dx, Dy, Dz)
    arr.markers.append(s)

    # 2) repere d'orientation de prise R_GRASP : 3 fleches (x=R, y=V, z=B), 8 cm
    cols = [(0, (1., 0., 0.)), (1, (0., 1., 0.)), (2, (0.2, 0.4, 1.))]
    for k, (col, rgb) in enumerate(cols):
        a = _m(10 + k, Marker.ARROW, *rgb, 1.0, 0.008, 0.016, 0.0)
        axis = Rg[:, col] * 0.08
        a.points = [_pt(Dx, Dy, Dz), _pt(Dx + axis[0], Dy + axis[1], Dz + axis[2])]
        arr.markers.append(a)

    # 3) descente ligne droite verticale (world) z_lift -> z_pick
    line = _m(20, Marker.LINE_STRIP, 0.6, 0.6, 0.6, 0.9, 0.004, 0., 0.)
    line.points = [_pt(Dx, Dy, z_lift), _pt(Dx, Dy, Dz)]
    arr.markers.append(line)
    # point haut = approche
    up = _m(21, Marker.SPHERE, 0.6, 0.6, 0.6, 0.7, 0.02, 0.02, 0.02)
    up.pose.position = _pt(Dx, Dy, z_lift)
    arr.markers.append(up)

    # 4) contour table physique
    tx, ty = o.TABLE_PHYS["x"], o.TABLE_PHYS["y"]
    tab = _m(30, Marker.LINE_STRIP, 1., 1., 1., 0.6, 0.004, 0., 0.)
    tab.points = [_pt(tx[0], ty[0], 0), _pt(tx[1], ty[0], 0),
                  _pt(tx[1], ty[1], 0), _pt(tx[0], ty[1], 0), _pt(tx[0], ty[0], 0)]
    arr.markers.append(tab)

    # 5) fenetre de tirage aleatoire autour de D (au z de prise)
    (xlo, xhi), (ylo, yhi) = o._window()
    win = _m(31, Marker.LINE_STRIP, 1., 0.9, 0.1, 0.9, 0.004, 0., 0.)
    win.points = [_pt(xlo, ylo, Dz), _pt(xhi, ylo, Dz), _pt(xhi, yhi, Dz),
                  _pt(xlo, yhi, Dz), _pt(xlo, ylo, Dz)]
    arr.markers.append(win)

    # 5b) cercles d'exclusion : keep-out D (orange) + keep-out base robot (rouge)
    def _circle(mid, cx, cy, cz, rad, rgb, n=48):
        c = _m(mid, Marker.LINE_STRIP, *rgb, 0.9, 0.004, 0., 0.)
        c.points = [_pt(cx + rad * np.cos(t), cy + rad * np.sin(t), cz)
                    for t in np.linspace(0, 2 * np.pi, n)]
        return c
    arr.markers.append(_circle(32, Dx, Dy, Dz, o.D_KEEPOUT, (1., 0.5, 0.)))     # 5 cm autour de D
    arr.markers.append(_circle(33, 0., 0., 0.0, o.BASE_KEEPOUT, (1., 0.2, 0.2)))  # 30 cm base robot

    # 5c) empreinte du socle 'base_robot' (bloc sous le bras) + marge = zone exclue (rouge)
    bhx = o._BLK_HALF[0] + o.BLOCK_MARGIN
    bhy = o._BLK_HALF[1] + o.BLOCK_MARGIN
    cx, cy = o._BLK_C
    blk = _m(34, Marker.LINE_STRIP, 1., 0.2, 0.2, 0.9, 0.004, 0., 0.)
    blk.points = [_pt(cx - bhx, cy - bhy, Dz), _pt(cx + bhx, cy - bhy, Dz),
                  _pt(cx + bhx, cy + bhy, Dz), _pt(cx - bhx, cy + bhy, Dz),
                  _pt(cx - bhx, cy - bhy, Dz)]
    arr.markers.append(blk)

    # 6) texte
    txt = _m(40, Marker.TEXT_VIEW_FACING, 1., 1., 1., 1.0, 0., 0., 0.035)
    txt.pose.position = _pt(Dx, Dy, Dz + 0.06)
    txt.text = "D (%.2f, %.2f, %.2f)" % (Dx, Dy, Dz)
    arr.markers.append(txt)
    return arr


def main():
    import signal
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, signal.SIG_IGN)
        except Exception:
            pass
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init()
    n = Node("roby_show_D")
    qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = n.create_publisher(MarkerArray, "/oracle_debug", qos)
    arr = build()
    n.get_logger().info("D_XYZ=(%.3f, %.3f, %.3f)  z_lift=%.3f  frame=%s" % (Dx, Dy, Dz, z_lift, FRAME))
    n.get_logger().info("R_GRASP z-axis (bleu) = %s" % np.round(Rg[:, 2], 3).tolist())
    pub.publish(arr)
    timer = n.create_timer(1.0, lambda: pub.publish(arr))  # re-latch periodique
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
