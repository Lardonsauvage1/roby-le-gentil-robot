#!/usr/bin/env python3
"""Capture la pose articulaire courante sous un nom dans ~/roby_poses.yaml.
Usage: capture_to_poses.py <nom>   (remplace si le nom existe deja)
Lit /joint_states (lecture seule, ne bouge rien)."""
import sys, os, time, re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
OUT = os.path.expanduser("~/roby_poses.yaml")


def main():
    if len(sys.argv) != 2:
        print("usage: capture_to_poses.py <nom>"); return 1
    name = sys.argv[1]
    rclpy.init(); n = Node("capture_to_poses"); got = {"v": None}

    def cb(m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in J):
            got["v"] = [round(float(d[j]), 4) for j in J]
    n.create_subscription(JointState, "/joint_states", cb, 10)
    t0 = time.time()
    while rclpy.ok() and got["v"] is None and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node(); rclpy.shutdown()
    if got["v"] is None:
        print("ECHEC: pas de /joint_states"); return 2
    vals = got["v"]
    line = "%s: [%s]\n" % (name, ", ".join("%.4f" % v for v in vals))
    # remplace une entree existante de meme nom, sinon ajoute
    lines = []
    if os.path.exists(OUT):
        lines = [l for l in open(OUT) if not re.match(r"^%s\s*:" % re.escape(name), l)]
    lines.append(line)
    open(OUT, "w").write("".join(lines))
    print("CAPTURE '%s' -> %s" % (name, OUT))
    print("  [%s]" % ", ".join("%.4f" % v for v in vals))
    print("  deg: " + ", ".join("%s=%.1f" % (j, vals[i] * 57.2958) for i, j in enumerate(J)))
    return 0


sys.exit(main())
