#!/usr/bin/env python3
"""Moniteur diagnostic : enregistre joint_2 (et joint_3) depuis /joint_states
pendant DUREE secondes, puis imprime la serie temporelle + un resume
(cible, valeur finale, overshoot, erreur residuelle). Throwaway tuning tool."""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DUREE = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
TARGET = float(sys.argv[2]) if len(sys.argv) > 2 else None  # cible joint_2 (rad)


class Mon(Node):
    def __init__(self):
        super().__init__("jmon")
        self.t0 = None
        self.samples = []  # (t, j2, j3)
        self.create_subscription(JointState, "/joint_states", self.cb, 50)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        d = dict(zip(msg.name, msg.position))
        self.samples.append((now - self.t0, d.get("joint_2"), d.get("joint_3")))


def main():
    rclpy.init()
    n = Mon()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.t0 is not None and n.samples and n.samples[-1][0] >= DUREE:
            break
    s = n.samples
    print(f"\n=== {len(s)} echantillons sur {DUREE}s ===")
    # serie sous-echantillonnee (~tous les 0.25s)
    last = -1
    for t, j2, j3 in s:
        if t - last >= 0.25:
            print(f"  t={t:5.2f}s  j2={j2*57.2958:7.2f}deg  j3={j3*57.2958:7.2f}deg")
            last = t
    j2s = [x[1] for x in s if x[1] is not None]
    if j2s and TARGET is not None:
        final = sum(j2s[-5:]) / len(j2s[-5:])
        # overshoot par rapport a la cible, dans le sens du mouvement
        start = j2s[0]
        if TARGET < start:
            peak = min(j2s)
            over = (start - peak) - (start - TARGET)
        else:
            peak = max(j2s)
            over = (peak - start) - (TARGET - start)
        print(f"\n=== RESUME joint_2 ===")
        print(f"  depart   = {start*57.2958:.2f} deg")
        print(f"  cible    = {TARGET*57.2958:.2f} deg")
        print(f"  pic      = {peak*57.2958:.2f} deg  (overshoot {over*57.2958:+.2f} deg)")
        print(f"  final    = {final*57.2958:.2f} deg")
        print(f"  ERREUR RESIDUELLE = {(TARGET-final)*57.2958:+.2f} deg")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
