#!/usr/bin/env python3
"""
roby_replay.py — Rejeu « faux reseau » d'un episode enregistre.

Rejoue TOUTE l'action enregistree = ce que produira le reseau a l'inference :
  - /joint_states enregistres -> TRAJECTOIRE de commande au /arm_controller (bras)
  - /gripper enregistres      -> RE-PUBLIES sur /gripper aux memes instants (pince)
Planificateur OFF. Le bras + la pince doivent REPRODUIRE l'episode => valide toute
la chaine enregistrement -> commande (joints ET pince, les deux sorties du modele).

SECURITE : le 1er point de la trajectoire ramene DOUCEMENT le bras (sur LEAD_IN s) a
la pose de depart du bag AVANT de rejouer.

Usage : python3 roby_replay.py <chemin_bag> [--rate 30] [--lead-in 3] [--go]
  sans --go : DRY (affiche ce qui serait envoye, NE BOUGE PAS).
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
from std_msgs.msg import Bool
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def read_bag(bag):
    """Retourne (joint_states, gripper) en temps ABSOLU (s)."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    js, gr = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/joint_states":
            m = deserialize_message(data, JointState)
            idx = {n: i for i, n in enumerate(m.name)}
            if all(j in idx for j in JOINTS):
                js.append((t / 1e9, [float(m.position[idx[j]]) for j in JOINTS]))
        elif topic == "/gripper":
            b = deserialize_message(data, Bool)
            gr.append((t / 1e9, bool(b.data)))
    return js, gr


def downsample(js, rate, t0):
    out, last = [], -1e9
    for (t, p) in js:
        if (t - last) >= (1.0 / rate) - 1e-6:
            out.append((t - t0, p)); last = t
    if out and abs(out[-1][0] - (js[-1][0] - t0)) > 1e-3:
        out.append((js[-1][0] - t0, js[-1][1]))
    return out


def build_traj(points, lead_in):
    traj = JointTrajectory(); traj.joint_names = list(JOINTS)
    p0 = JointTrajectoryPoint()
    p0.positions = [float(v) for v in points[0][1]]
    p0.time_from_start = Duration(sec=int(lead_in), nanosec=int((lead_in % 1) * 1e9))
    traj.points.append(p0)
    for (t, pos) in points[1:]:
        pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in pos]
        tt = lead_in + t
        pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
        traj.points.append(pt)
    return traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--lead-in", type=float, default=3.0)
    ap.add_argument("--go", action="store_true", help="requis pour BOUGER (bras + pince)")
    args = ap.parse_args()

    js, gr = read_bag(args.bag)
    if len(js) < 2:
        print("Bag sans /joint_states exploitable."); return 1
    t0 = js[0][0]
    pts = downsample(js, args.rate, t0)
    grip = [(t - t0, v) for (t, v) in gr]          # pince, meme reference temps que le bras
    dur = pts[-1][0]
    print(f"Bag: {len(js)} joint_states -> {len(pts)} pts @ {args.rate:.0f}Hz, {len(gr)} consignes pince, rejeu {dur:.1f}s")
    print(f"depart j=[{', '.join('%.3f'%v for v in pts[0][1])}]")
    print("pince a rejouer : " + ", ".join(f"{'FERME' if v else 'OUVRE'}@{t:.1f}s" for t, v in grip))
    if not args.go:
        print("DRY (pas de --go) : rien envoye. Ajoute --go pour bouger le bras + la pince.")
        return 0

    rclpy.init()
    node = Node("roby_replay")
    ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    grip_pub = node.create_publisher(Bool, "/gripper", 10)
    if not ac.wait_for_server(timeout_sec=10):
        print("arm_controller absent (stack up ?)"); return 1

    traj = build_traj(pts, args.lead_in)
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    print(f"Rejeu : lead-in {args.lead_in:.0f}s vers pose depart, puis {dur:.1f}s (bras + pince)...")
    fut = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh or not gh.accepted:
        print("Trajectoire REFUSEE par le controleur."); return 1

    rf = gh.get_result_async()
    t_start = time.monotonic()           # ~debut execution (lead-in inclus)
    gi = 0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        el = time.monotonic() - t_start
        while gi < len(grip) and el >= args.lead_in + grip[gi][0]:
            v = grip[gi][1]
            grip_pub.publish(Bool(data=v))
            print(f"  [pince] {'FERME' if v else 'OUVRE'} @ {el:.1f}s")
            gi += 1
        if rf.done():
            break
    # publie les consignes pince restantes (si la traj finit avant)
    for t, v in grip[gi:]:
        grip_pub.publish(Bool(data=v)); print(f"  [pince] {'FERME' if v else 'OUVRE'} (fin)")
    print("Rejeu termine (bras + pince ont reproduit l'episode enregistre).")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
