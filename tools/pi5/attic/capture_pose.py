#!/usr/bin/env python3
"""Capture la pose articulaire courante (joint_1..5) sous un nom, dans ~/demo_poses.yaml.
Usage: python3 capture_pose.py <nom_pose>
"""
import sys, os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
OUT = os.path.expanduser("~/demo_poses.yaml")

class Cap(Node):
    def __init__(self, name):
        super().__init__("capture_pose")
        self.name = name; self.got = False
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
    def cb(self, msg):
        if self.got: return
        m = dict(zip(msg.name, msg.position))
        if not all(j in m for j in JOINTS): return
        vals = [round(float(m[j]), 4) for j in JOINTS]
        with open(OUT, "a") as f:
            f.write("%s: [%s]\n" % (self.name, ", ".join("%.4f" % v for v in vals)))
        print("capture %s: %s" % (self.name, vals))
        self.got = True

def main():
    if len(sys.argv) != 2:
        print("usage: capture_pose.py <nom_pose>"); return 1
    rclpy.init(); n = Cap(sys.argv[1])
    import time; t0 = time.time()
    while rclpy.ok() and not n.got and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node(); rclpy.shutdown()
    return 0 if n.got else 2

sys.exit(main())
