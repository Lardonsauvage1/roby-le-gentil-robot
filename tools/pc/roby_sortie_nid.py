#!/usr/bin/env python3
"""roby_sortie_nid.py — Sort la tete du nid en rejouant une trajectoire nettoyee.

Envoie les waypoints de ~/roby_sortie_nid.yaml a l'/arm_controller
(FollowJointTrajectory). Trajectoire = teleop demontree par Sam puis nettoyee
(dwell-removal + Douglas-Peucker), 5 waypoints monotones nid -> sortie.

SECURITE :
  - 1er point atteint DOUCEMENT (LEAD_IN s) : pas de saut au demarrage.
  - Vitesse joint bornee (SAFE_VEL rad/s) : lent et sur.
  - --dry (defaut) : affiche la trajectoire, NE BOUGE PAS. --go pour BOUGER.
  - --reverse : sens inverse (sortie -> nid) pour re-docker dans le nid.

Prerequis : stack up (RobySystem, archi B). Pour --go : bras AU NID
(ou a la pose de sortie si --reverse). Open-loop => partir de la bonne pose.
"""
import argparse
import math
import os
import sys
import time

import yaml
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
YAML = os.path.expanduser("~/roby_sortie_nid.yaml")
SAFE_VEL = 0.15   # rad/s max par joint (lent / sur)
# 3 s suffisent depuis une pose proche du 1er point ; l'allonger si on part de loin
# (sinon le rattrapage vers wps[0] depasse SAFE_VEL).
LEAD_IN = float(os.environ.get("ROBY_LEAD_IN", 3.0))
MIN_SEG = 0.8     # s minimum par segment
# Le nid utilise le MEME connecteur qu'un outil : verrou engage = tete retenue par le
# berceau. Tirer dessus verrouille force sur le changeur d'outil. On deverrouille donc
# TOUJOURS avant de bouger, dans les DEUX sens (re-docker exige aussi le verrou ouvert).
UNLOCK_SETTLE = float(os.environ.get("ROBY_UNLOCK_SETTLE", 1.5))  # s de course du servo


def build(wps):
    traj = JointTrajectory()
    traj.joint_names = JOINTS
    t = LEAD_IN
    p0 = JointTrajectoryPoint()
    p0.positions = [float(x) for x in wps[0]]
    p0.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    traj.points.append(p0)
    for a, b in zip(wps, wps[1:]):
        dmax = max(abs(b[k] - a[k]) for k in range(5))
        t += max(MIN_SEG, dmax / SAFE_VEL)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in b]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points.append(pt)
    return traj, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true", help="requis pour BOUGER le bras")
    ap.add_argument("--reverse", action="store_true", help="sens inverse : sortie -> nid")
    a = ap.parse_args()

    wps = yaml.safe_load(open(YAML))["waypoints"]
    if a.reverse:
        wps = list(reversed(wps))
    traj, total = build(wps)

    deg = lambda r: r * 180 / math.pi
    sens = "REVERSE (sortie -> nid)" if a.reverse else "sortie du nid (nid -> sortie)"
    print(f"{sens} : {len(wps)} waypoints, ~{total:.1f}s a SAFE_VEL={SAFE_VEL} rad/s")
    for pt in traj.points:
        ts = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
        print(f"  t={ts:5.1f}s  " + " ".join(f"{deg(v):7.1f}" for v in pt.positions) + " deg")

    print("  (avec --go : DEVERROUILLAGE de la tete d'abord, puis mouvement)")

    if not a.go:
        print("\n[DRY] rien envoye. Ajoute --go pour BOUGER (bras au nid + validation Sam).")
        return 0

    rclpy.init()
    node = rclpy.create_node("roby_sortie_nid")
    ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=5.0):
        print("❌ arm_controller absent (stack up ?)")
        return 1

    # --- DEVERROUILLAGE OBLIGATOIRE, avant tout mouvement -------------------
    # Fail-safe : sans abonne a /head_lock (= RobySystem), on ne PEUT PAS garantir que
    # le verrou s'ouvre => on refuse de bouger plutot que de tirer sur le berceau.
    lock_pub = node.create_publisher(Bool, "/head_lock", 10)
    t0 = time.monotonic()
    while lock_pub.get_subscription_count() < 1:
        if time.monotonic() - t0 > 5.0:
            print("❌ /head_lock sans abonne : deverrouillage NON garanti -> ON NE BOUGE PAS.")
            print("   (RobySystem tourne-t-il ? la stack est-elle bien en archi B ?)")
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)
    lock_pub.publish(Bool(data=False))
    print(f"verrou : DEVERROUILLE (/head_lock false), attente {UNLOCK_SETTLE:.1f}s de course servo...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < UNLOCK_SETTLE:
        rclpy.spin_once(node, timeout_sec=0.05)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    fut = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        print("❌ goal REFUSE par l'arm_controller")
        return 1
    print("goal accepte, execution...")
    rfut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rfut)
    # Verifier le RESULTAT : un goal avorte (tolerance, arret) s'affichait « termine »
    # comme un succes (revue du 2026-09-13).
    res = rfut.result()
    code = getattr(getattr(res, "result", None), "error_code", None)
    from action_msgs.msg import GoalStatus
    ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED and code == 0
    print("✅ termine." if ok else
          "❌ trajectoire NON achevee (statut %s, error_code %s) : verifier la pose du bras"
          % (getattr(res, "status", "?"), code))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


sys.exit(main())
