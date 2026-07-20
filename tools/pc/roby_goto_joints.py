#!/usr/bin/env python3
"""roby_goto_joints.py — va a une pose articulaire par trajectoire DIRECTE
(/arm_controller), sans MoveIt. Utile quand MoveIt refuse (start state out of
bounds). PAS d'anti-collision -> reserver aux petits deplacements surs.

Usage :
  roby_goto_joints.py j1 j2 j3 j4 j5 [--go]          # cible absolue
  roby_goto_joints.py --set j3=0.63 [--go]           # garde la pose courante, override certains joints
Sans --go = DRY (affiche, ne bouge pas). Vitesse bornee SAFE_VEL, lead-in doux.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import os
sys.path.insert(0, os.path.expanduser("~"))
from roby_oracle import LIMITS          # butees articulaires = source unique

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
SAFE_VEL = 0.15   # rad/s max par joint
LEAD_IN = 2.5     # s pour atteindre la cible (doux)
MIN_T = 2.0


def check_limits(tgt):
    """Refuse une cible hors butee. AJOUTE le 2026-07-20 : ce script prend 5
    flottants en argv et ecrit DIRECTEMENT au controleur, sans passer par le garde.
    En dessous, le SafetyMonitor C++ ne clampe qu'a +/-pi (les vraies butees URDF
    n'y sont pas lues) : joint_3 pouvait donc partir 143 deg au-dela de sa butee
    mecanique sur une simple faute de frappe (0.30 tape 3.0). Ici on est la SEULE
    barriere avant les moteurs.
    Retourne la liste des violations (vide = cible acceptable)."""
    bad = []
    for i, n in enumerate(JOINTS):
        lo, hi = LIMITS[n]
        if not (lo <= tgt[i] <= hi):
            bad.append(f"{n} = {tgt[i]:+.4f} hors butee [{lo:+.3f}, {hi:+.3f}]")
    return bad


class Goto(Node):
    def __init__(self):
        super().__init__("roby_goto_joints")
        self.cur = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.cur = [float(d[j]) for j in JOINTS]

    def wait(self):
        t0 = time.time()
        while self.cur is None and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("vals", nargs="*")
    ap.add_argument("--set", action="append", default=[], help="jN=val override (garde le reste)")
    ap.add_argument("--go", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Goto()
    cur = n.wait()
    if cur is None:
        print("pas de /joint_states"); return 1

    if a.set:
        tgt = list(cur)
        for s in a.set:
            k, v = s.split("=")
            tgt[JOINTS.index(k if k.startswith("joint_") else "joint_" + k[-1])] = float(v)
    elif len(a.vals) == 5:
        tgt = [float(x) for x in a.vals]
    else:
        print("donne 5 valeurs OU --set jN=val"); return 1

    dmax = max(abs(tgt[i] - cur[i]) for i in range(5))
    t = max(MIN_T, LEAD_IN, dmax / SAFE_VEL)
    print("actuel :", ["%.4f" % v for v in cur])
    print("cible  :", ["%.4f" % v for v in tgt])
    print("delta max = %.4f rad  -> duree %.1fs" % (dmax, t))

    # BARRIERE : rien en aval ne rattrapera une cible hors butee (ni le garde,
    # qui est court-circuite, ni le C++ qui clampe seulement a +/-pi).
    bad = check_limits(tgt)
    if bad:
        print("\n❌ CIBLE REFUSEE — hors butee articulaire :")
        for b in bad:
            print("   " + b)
        print("   (rien en aval ne l'aurait arretee : ce script ecrit direct au controleur)")
        return 1
    if not a.go:
        print("DRY : ajoute --go pour bouger."); return 0
    if not n.ac.wait_for_server(timeout_sec=10):
        print("arm_controller absent"); return 1
    traj = JointTrajectory(); traj.joint_names = JOINTS
    pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in tgt]
    pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    traj.points.append(pt)
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    fut = n.ac.send_goal_async(goal); rclpy.spin_until_future_complete(n, fut)
    gh = fut.result()
    if not gh or not gh.accepted:
        print("trajectoire refusee"); return 1
    print("goal accepte, mouvement...")
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf)
    print("termine.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
