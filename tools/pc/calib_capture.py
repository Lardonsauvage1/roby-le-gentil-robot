#!/usr/bin/env python3
"""Capture un point de calibration : lit joints + TCP sim, compare au coin reel
connu, enregistre dans ~/calib_points.yaml.
Usage: python3 calib_capture.py <nom_coin>
Coins connus (repere base robot, +X avant / +Y gauche / +Z haut) :
  socle (base_robot, dessus Z=-0.060) : socle_AvG socle_AvD socle_ArG socle_ArD
  table (plan_travail, dessus Z=-0.190): table_AvG table_AvD table_ArG table_ArD
"""
import sys, os, time, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
REAL = {
    "socle_AvG": (0.258, 0.530, -0.060), "socle_AvD": (0.258, -0.345, -0.060),
    "socle_ArG": (-0.258, 0.530, -0.060), "socle_ArD": (-0.258, -0.345, -0.060),
    "table_AvG": (1.042, 0.542, -0.190), "table_AvD": (1.042, -0.217, -0.190),
    "table_ArG": (-0.258, 0.542, -0.190), "table_ArD": (-0.258, -0.217, -0.190),
}
OUT = os.path.expanduser("~/calib_points.yaml")


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in REAL:
        print("usage: calib_capture.py <coin>  parmi:", ", ".join(REAL)); return 2
    name = sys.argv[1]; real = REAL[name]
    rclpy.init(); n = Node("calibrec")
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    js = {"v": None}; xyz = None; t0 = time.time()

    def cb(m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            js["v"] = [float(d[j]) for j in JOINTS]
    n.create_subscription(JointState, "/joint_states", cb, 10)
    while time.time() - t0 < 6:
        rclpy.spin_once(n, timeout_sec=0.2)
        if js["v"] is not None:
            try:
                # FRAME DE CONTACT = tcp : la table bien mesuree prouve que le
                # contact physique coincide avec le `tcp` du modele (9.5mm) et
                # PAS link_gripper (90mm off). Le `tcp` URDF = vraie face coupleur.
                t = buf.lookup_transform("world", "tcp", rclpy.time.Time())
                tr = t.transform.translation; xyz = (tr.x, tr.y, tr.z); break
            except Exception:
                pass
    if js["v"] and xyz:
        j = js["v"]; err = tuple(real[i] - xyz[i] for i in range(3))
        line = ("%s: joints=[%s] sim_tcp=[%.4f,%.4f,%.4f] real_tcp=[%.4f,%.4f,%.4f] "
                "err=[%+.4f,%+.4f,%+.4f]" % (name, ",".join("%.4f" % v for v in j),
                *xyz, *real, *err))
        open(OUT, "a").write(line + "\n")
        print("ENREGISTRE:", name)
        print("  angles :", ", ".join("%s=%.2f" % (jn, j[i] * 57.2958) for i, jn in enumerate(JOINTS)))
        print("  sim    : x=%+.4f y=%+.4f z=%+.4f" % xyz)
        print("  reel   : x=%+.4f y=%+.4f z=%+.4f" % real)
        print("  ECART  : dx=%+.1f dy=%+.1f dz=%+.1f mm (norme %.1f mm)" % (
            err[0] * 1000, err[1] * 1000, err[2] * 1000, math.dist((0, 0, 0), err) * 1000))
    else:
        print("ECHEC: pas de joint_states/tf")
    n.destroy_node(); rclpy.shutdown()


main()
