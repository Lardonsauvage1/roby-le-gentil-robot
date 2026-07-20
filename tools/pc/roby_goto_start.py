#!/usr/bin/env python3
"""roby_goto_start.py — amene le bras a la POSE DE DEPART d'un episode du dataset,
pour que le reseau demarre IN-DISTRIBUTION (aerien, pince ouverte, pomme visible).

Lit le 1er /joint_states de l'episode -> 1 point de trajectoire lent (FollowJointTrajectory,
meme canal eprouve que roby_sortie_nid.sh). Duree calculee pour ne pas depasser SAFE_VEL.

  roby_goto_start.py <ep>            # DRY : affiche cible/duree, NE BOUGE PAS
  roby_goto_start.py <ep> --go       # BOUGE
  roby_goto_start.py --median <batch>  # cible = depart MEDIAN du batch (le plus typique)
"""
import argparse, glob, os, sys, time
sys.path.insert(0, os.path.expanduser("~"))
import numpy as np
from roby_oracle import fkT, LIMITS

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
SAFE_VEL = 0.15          # rad/s : meme prudence que la sortie du nid
MIN_DUR = 4.0


def first_q(ep):
    r = SequentialReader()
    r.open(StorageOptions(uri=ep, storage_id="mcap"), ConverterOptions("", ""))
    while r.has_next():
        t, d, ts = r.read_next()
        if t == "/joint_states":
            m = deserialize_message(d, JointState)
            dd = dict(zip(m.name, m.position))
            if all(j in dd for j in J):
                return np.array([float(dd[j]) for j in J])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="chemin episode (ep_xxx) ou batch si --median")
    ap.add_argument("--median", action="store_true", help="cible = depart median du batch")
    ap.add_argument("--go", action="store_true")
    a = ap.parse_args()
    tgt = os.path.expanduser(a.target)

    if a.median:
        eps = sorted(e for e in glob.glob(tgt + "/ep_*") if os.path.isdir(e))
        qs = [q for q in (first_q(e) for e in eps) if q is not None]
        med = np.median(np.array(qs), axis=0)
        # episode REEL le plus proche du median (on evite une pose synthetique hors-variete)
        idx = int(np.argmin([np.linalg.norm(q - med) for q in qs]))
        q_t = qs[idx]
        print(f"{len(qs)} episodes ; depart le plus typique = {os.path.basename(eps[idx])}")
    else:
        q_t = first_q(tgt)
        if q_t is None:
            print("pas de /joint_states dans l'episode"); return 1
        print(f"depart de {os.path.basename(tgt)}")

    for k, n in enumerate(J):
        lo, hi = LIMITS[n]
        if not (lo <= q_t[k] <= hi):
            print(f"❌ {n}={q_t[k]:.3f} hors butee [{lo},{hi}]"); return 1

    p = fkT(q_t)[:3, 3]
    print(f"cible q   = [{' '.join('%+.4f' % v for v in q_t)}]")
    print(f"cible TCP = [{p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}] m")

    rclpy.init()
    node = Node("roby_goto_start")
    cur = {}

    def _js(m):
        dd = dict(zip(m.name, m.position))
        if all(j in dd for j in J):
            cur["q"] = np.array([float(dd[j]) for j in J])

    node.create_subscription(JointState, "/joint_states", _js, 10)
    t0 = time.monotonic()
    while "q" not in cur and time.monotonic() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "q" not in cur:
        print("❌ pas de /joint_states : stack lancee ?"); node.destroy_node(); rclpy.shutdown(); return 1

    q0 = cur["q"]
    dmax = float(np.max(np.abs(q_t - q0)))
    dur = max(MIN_DUR, dmax / SAFE_VEL)
    print(f"actuel    = [{' '.join('%+.4f' % v for v in q0)}]")
    print(f"ecart max = {np.degrees(dmax):.1f} deg -> duree {dur:.1f} s a {SAFE_VEL} rad/s")

    if not a.go:
        print("\n[DRY] rien envoye. Ajoute --go pour BOUGER.")
        node.destroy_node(); rclpy.shutdown(); return 0

    ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=10):
        print("❌ arm_controller absent"); node.destroy_node(); rclpy.shutdown(); return 1
    traj = JointTrajectory(); traj.joint_names = list(J)
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in q_t]
    pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
    traj.points = [pt]
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    fut = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh or not gh.accepted:
        print("❌ trajectoire REFUSEE"); node.destroy_node(); rclpy.shutdown(); return 1
    print("goal accepte, execution...")
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    print("✅ arrive au depart d'episode.")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
