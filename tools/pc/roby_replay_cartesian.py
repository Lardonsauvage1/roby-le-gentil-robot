#!/usr/bin/env python3
"""roby_replay_cartesian.py — rejoue un episode A PARTIR du topic CARTESIEN /tcp_pose.
Lit /tcp_pose [x,y,z,rvx,rvy,rvz] -> reconstruit les joints par DLS (chaine, comme au
deploiement d'un modele cartesien) -> rejoue au /arm_controller. Valide toute la chaine
cartesien -> IK -> mouvement. La pince (/gripper) est rejouee aux memes instants.

SECURITE : (1) verifie la reconstruction (FK doit recoller < FK_TOL, pas de saut > JUMP_TOL)
AVANT de bouger ; refuse sinon. (2) 1er point ramene DOUCEMENT au depart (LEAD_IN).
Usage : roby_replay_cartesian.py <bag_cartesien> [--rate 20] [--lead-in 4] [--go]
  sans --go = DRY (reconstruit + verifie + affiche, NE BOUGE PAS).
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.expanduser("~"))
import numpy as np
from roby_oracle import fkT, D_JOINTS
from roby_tool_pickup import dls, rotvec

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, Float64MultiArray
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
FK_TOL = 0.006      # m : ecart max FK(joint_reconstruit) vs tcp stocke
JUMP_TOL = 0.30     # rad : saut articulaire max entre 2 points consecutifs


def rv_to_R(rv):
    rv = np.array(rv, float); th = np.linalg.norm(rv)
    if th < 1e-9: return np.eye(3)
    k = rv / th; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def read_cart(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    tcp, gr = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/tcp_pose":
            m = deserialize_message(data, Float64MultiArray)
            if len(m.data) >= 6: tcp.append((t / 1e9, np.array(m.data[:6])))
        elif topic == "/gripper":
            gr.append((t / 1e9, bool(deserialize_message(data, Bool).data)))
    return tcp, gr


def downsample(seq, rate, t0):
    out, last = [], -1e9
    for (t, v) in seq:
        if (t - last) >= (1.0 / rate) - 1e-6:
            out.append((t - t0, v)); last = t
    if out and abs(out[-1][0] - (seq[-1][0] - t0)) > 1e-3:
        out.append((seq[-1][0] - t0, seq[-1][1]))
    return out


def reconstruct(tcp_pts):
    """tcp -> joints par DLS chaine. 1er seed = azimut du point (grasp-azimut).
    Retourne (points[(t,joints)], worst_fk_mm, worst_jump_rad)."""
    out = []; worst_fk = 0.0; worst_jump = 0.0; prev = None
    for k, (t, tcp) in enumerate(tcp_pts):
        p = tcp[:3]; R = rv_to_R(tcp[3:6])
        if prev is None:
            seed = np.array(D_JOINTS, float); seed[0] = float(np.arctan2(p[1], p[0]))
        else:
            seed = prev
        j = np.array(dls(seed, p, R, iters=25, w_ori=1.0), float)
        fk_err = np.linalg.norm(fkT(j)[:3, 3] - p)
        worst_fk = max(worst_fk, fk_err)
        if prev is not None:
            worst_jump = max(worst_jump, float(np.max(np.abs(j - prev))))
        out.append((t, j)); prev = j
    return out, worst_fk * 1000, worst_jump


def build_traj(points, lead_in):
    traj = JointTrajectory(); traj.joint_names = list(JOINTS)
    p0 = JointTrajectoryPoint(); p0.positions = [float(v) for v in points[0][1]]
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
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--lead-in", type=float, default=4.0)
    ap.add_argument("--go", action="store_true")
    a = ap.parse_args()

    tcp, gr = read_cart(a.bag)
    if len(tcp) < 2:
        print("Bag sans /tcp_pose exploitable."); return 1
    t0 = tcp[0][0]
    pts_tcp = downsample(tcp, a.rate, t0)
    pts, worst_fk, worst_jump = reconstruct(pts_tcp)
    grip = [(t - t0, v) for (t, v) in gr]
    dur = pts[-1][0]
    print(f"/tcp_pose: {len(tcp)} -> {len(pts)} pts @ {a.rate:.0f}Hz, rejeu {dur:.1f}s")
    print(f"reconstruction DLS : FK recolle a {worst_fk:.2f} mm max (tol {FK_TOL*1000:.0f}), "
          f"saut articulaire max {np.degrees(worst_jump):.1f} deg (tol {np.degrees(JUMP_TOL):.0f})")
    print(f"depart j=[{', '.join('%.3f'%v for v in pts[0][1])}]")
    ok = worst_fk <= FK_TOL * 1000 and worst_jump <= JUMP_TOL
    if not ok:
        print("❌ RECONSTRUCTION NON FIABLE -> je NE rejoue PAS (securite)."); return 1
    print("✅ reconstruction fiable.")
    if not a.go:
        print("DRY : rien envoye. Ajoute --go pour bouger (bras + pince)."); return 0

    rclpy.init(); node = Node("roby_replay_cart")
    ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    grip_pub = node.create_publisher(Bool, "/gripper", 10)
    if not ac.wait_for_server(timeout_sec=10):
        print("arm_controller absent"); return 1
    traj = build_traj(pts, a.lead_in)
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    print(f"Rejeu : lead-in {a.lead_in:.0f}s vers depart, puis {dur:.1f}s...")
    fut = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh or not gh.accepted:
        print("Trajectoire REFUSEE."); return 1
    rf = gh.get_result_async(); t_start = time.monotonic(); gi = 0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        el = time.monotonic() - t_start
        while gi < len(grip) and el >= a.lead_in + grip[gi][0]:
            grip_pub.publish(Bool(data=grip[gi][1]))
            print(f"  [pince] {'FERME' if grip[gi][1] else 'OUVRE'} @ {el:.1f}s"); gi += 1
        if rf.done(): break
    for t, v in grip[gi:]:
        grip_pub.publish(Bool(data=v))
    print("Rejeu cartesien termine.")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
