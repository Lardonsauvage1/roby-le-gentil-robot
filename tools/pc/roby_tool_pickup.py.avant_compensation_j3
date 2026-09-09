#!/usr/bin/env python3
"""Séquence de prise d'outil (pince) — lignes droites en IK amortie DLS maison
(robuste aux singularités 5-DOF) + mouvement libre en MoveIt (anti-collision).

Séquence (depuis le NID) :
  1. déverrouille (/head_lock false)
  2. LIGNE DROITE (DLS) : monte +10 cm  -> approche_nid
  3. LIBRE (MoveIt, anti-collision) -> approche_changeur
  4. ouvre la pince (/gripper false)
  5. LIGNE DROITE (DLS) : descend -> changeur_outil
  6. verrouille (/head_lock true)
  7. LIGNE DROITE (DLS) : monte +10 cm -> approche_changeur
  8. ferme puis ouvre la pince

Lignes droites : FK maison -> Jacobienne num -> DLS -> JointTrajectory ->
/arm_controller/follow_joint_trajectory. Libre : /move_action. Repère = link_gripper.
--dry : valide tout (DLS + plan libre) sans bouger. LE ROBOT BOUGE en réel.
"""
import os
import sys
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, RobotState)

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GROUP = "arm"
APPROACH = 0.10            # hauteur d'approche (m)
CART_SPEED = 0.03          # vitesse cartesienne des lignes droites (m/s)
STEP = 0.005               # pas d'interpolation des lignes droites (m)
FREE_VEL = 0.50            # facteur vitesse du mouvement libre MoveIt (x5 vs le run lent 0.10)
DESCENT_W_ORI = 1.0        # poids orientation des LIGNES DROITES (1=strict, changeur d'outil).
                           # L'oracle le baisse (~0.2) pour prioriser la position sur toute la table.
LIMITS = {"joint_1": (-3.14159, 3.14159), "joint_2": (-1.6, 2.1),
          "joint_3": (-3.0, 0.65), "joint_4": (-3.1416, 3.1416),
          "joint_5": (-1.6, 1.6)}
POSES_FILE = os.path.expanduser("~/roby_poses.yaml")


# ---------- FK / Jacobienne / DLS (repère link_gripper) ----------
def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def H(R, t): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def fkT(j):
    j1, j2, j3, j4, j5 = j
    T = np.eye(4)
    T = T @ H(Rz(j1), [0, 0, 0.02])
    T = T @ H(Ry(j2), [0.024031, 0, 0.202992])
    T = T @ H(Ry(j3), [-0.015224, 0, 0.441653])
    T = T @ H(Rx(j4), [0.119473, 0, 0.029716])
    T = T @ H(Ry(j5), [0.321516, 0, 0])
    T = T @ H(np.eye(3), [0.06, 0, 0])     # link_gripper
    return T


def fk_pos(j):
    return fkT(j)[:3, 3]


def rotvec(Rm):
    ang = np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))
    if ang < 1e-8:
        return np.zeros(3)
    return ang / (2 * np.sin(ang)) * np.array(
        [Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])


def jac(j, eps=1e-5):
    T0 = fkT(j); p0 = T0[:3, 3]; R0 = T0[:3, :3]; Jm = np.zeros((6, 5))
    for k in range(5):
        jj = np.array(j, float); jj[k] += eps; T1 = fkT(jj)
        Jm[:3, k] = (T1[:3, 3] - p0) / eps
        Jm[3:, k] = R0 @ rotvec(R0.T @ T1[:3, :3]) / eps
    return Jm


def dls(j, target_p, target_R, lam=0.06, iters=8, w_ori=1.0):
    # w_ori < 1 => on prioritise la POSITION : l'erreur d'orientation compte moins,
    # le solveur tient d'abord le xyz (utile en 5 DOF ou position+orientation
    # exactes sont incompatibles loin de la zone centrale). w_ori=1 => comportement
    # d'origine (orientation stricte, garde pour le changeur d'outil).
    j = np.array(j, float)
    for _ in range(iters):
        T = fkT(j); p = T[:3, 3]; R = T[:3, :3]
        e = np.concatenate([target_p - p, w_ori * (R @ rotvec(R.T @ target_R))])
        Jm = jac(j)
        dq = Jm.T @ np.linalg.inv(Jm @ Jm.T + lam ** 2 * np.eye(6)) @ e
        j = j + dq
    return j


def load_pose(name):
    import yaml
    for p in [POSES_FILE]:
        if os.path.exists(p):
            d = yaml.safe_load(open(p)) or {}
            if name in d and len(d[name]) == 5:
                return [float(x) for x in d[name]]
    return None


class Pickup(Node):
    def __init__(self, dry=False):
        super().__init__("roby_tool_pickup")
        self.dry = dry
        self.cur = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.lock_pub = self.create_publisher(Bool, "/head_lock", 10)
        self.grip_pub = self.create_publisher(Bool, "/gripper", 10)
        self.traj_ac = ActionClient(self, FollowJointTrajectory,
                                    "/arm_controller/follow_joint_trajectory")
        self.move_ac = ActionClient(self, MoveGroup, "/move_action")

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in J):
            self.cur = [float(d[j]) for j in J]

    def log(self, m):
        self.get_logger().info(m)

    def wait_state(self):
        t0 = time.time()
        while self.cur is None and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.cur

    def rs(self, joints):
        s = RobotState(); s.joint_state = JointState(name=list(J), position=list(joints))
        return s

    def _spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ---------- ligne droite DLS ----------
    def straight(self, target_p, label="", dry_start=None):
        start = dry_start if (self.dry and dry_start is not None) else self.wait_state()
        start = np.array(start, float)
        p0 = fk_pos(start); keepR = fkT(start)[:3, :3]
        dist = np.linalg.norm(np.array(target_p) - p0)
        N = max(2, int(math.ceil(dist / STEP)))
        wps = []; j = start.copy(); track = 0.0
        for i in range(1, N + 1):
            wp = p0 + (i / N) * (np.array(target_p) - p0)
            j = dls(j, wp, keepR, w_ori=DESCENT_W_ORI)
            track = max(track, np.linalg.norm(fk_pos(j) - wp))
            # garde-fous : butées + pas de saut articulaire
            for k, jn in enumerate(J):
                lo, hi = LIMITS[jn]
                if not (lo - 0.02 <= j[k] <= hi + 0.02):
                    self.log("  LIGNE DROITE %s : ECHEC (joint %s hors butee)" % (label, jn))
                    return False
            if wps and max(abs(j[k] - wps[-1][k]) for k in range(5)) > 0.25:
                self.log("  LIGNE DROITE %s : ECHEC (saut articulaire)" % label)
                return False
            wps.append(j.copy())
        if track > 0.01:
            self.log("  LIGNE DROITE %s : ECHEC (suivi %.0fmm > 10mm)" % (label, track * 1000))
            return False
        total = max(1.5, dist / CART_SPEED)
        self.log("  LIGNE DROITE %s (DLS) : %d pts, suivi %.1fmm, %.1fs%s"
                 % (label, N, track * 1000, total, " [DRY]" if self.dry else ""))
        if self.dry:
            return True
        return self._exec_traj(wps, total)

    def _exec_traj(self, wps, total):
        if not self.traj_ac.wait_for_server(timeout_sec=10):
            self.log("  arm_controller absent"); return False
        traj = JointTrajectory(); traj.joint_names = list(J)
        for i, wp in enumerate(wps):
            pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in wp]
            t = total * (i + 1) / len(wps)
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(pt)
        goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
        fut = self.traj_ac.send_goal_async(goal); rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.log("  trajectoire refusee"); return False
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        return True

    # ---------- mouvement libre MoveIt ----------
    def free_to(self, joints, label="", dry_start=None):
        if not self.move_ac.wait_for_server(timeout_sec=10):
            self.log("  move_group absent"); return False
        req = MotionPlanRequest()
        req.group_name = GROUP; req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = FREE_VEL
        req.max_acceleration_scaling_factor = FREE_VEL
        if self.dry and dry_start is not None:
            req.start_state = self.rs(dry_start)
        c = Constraints()
        for jn, val in zip(J, joints):
            jc = JointConstraint(); jc.joint_name = jn; jc.position = float(val)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal = MoveGroup.Goal(); goal.request = req
        opt = PlanningOptions(); opt.plan_only = self.dry
        opt.planning_scene_diff.is_diff = True
        opt.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opt
        self.log("  LIBRE %s : planif + exec (vel=%.2f)..." % (label, FREE_VEL))
        fut = self.move_ac.send_goal_async(goal); rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.log("  goal refuse"); return False
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        ok = rf.result().result.error_code.val == 1
        self.log("  LIBRE %s : %s" % (label, "OK" if ok else "ECHEC"))
        return ok

    def lock(self, v):
        if not self.dry:
            self.lock_pub.publish(Bool(data=v))
        self.log("  %s%s" % ("VERROU" if v else "DEVERROU", " [DRY]" if self.dry else ""))
        self._spin(0.3 if self.dry else 1.5)

    def grip(self, close):
        if not self.dry:
            self.grip_pub.publish(Bool(data=close))
        self.log("  pince %s%s" % ("FERME" if close else "OUVRE", " [DRY]" if self.dry else ""))
        self._spin(0.3 if self.dry else 1.5)

    # ---------- approche changeur (config "montee", robuste descente) ----------
    def approach_changeur(self, chg):
        """Config approche = DLS montee +10cm depuis le changeur (= depuis cette
        config la descente DLS marche, on ferme la boucle)."""
        p0 = fk_pos(chg); keepR = fkT(chg)[:3, :3]
        tgt = p0 + np.array([0, 0, APPROACH])
        j = np.array(chg, float)
        N = max(2, int(math.ceil(APPROACH / STEP)))
        for i in range(1, N + 1):
            j = dls(j, p0 + (i / N) * (tgt - p0), keepR)
        return j

    def run(self):
        nid = load_pose("nid"); chg = load_pose("changeur_outil")
        if not nid or not chg:
            self.log("Poses 'nid'/'changeur_outil' manquantes."); return
        self.wait_state()
        nid_p = fk_pos(nid); chg_p = fk_pos(chg)
        appr_chg = self.approach_changeur(chg)
        appr_chg_pos = fk_pos(appr_chg)
        self._save("approche_changeur", appr_chg)
        appr_nid = self.approach_changeur(nid)   # meme methode pour le nid
        self._save("approche_nid", appr_nid)

        if not self.dry:
            cur = self.wait_state()
            ec = max(abs(cur[i] - nid[i]) for i in range(5)) if cur else 9
            if ec > 0.08:
                self.log("ABANDON : bras pas au nid (ecart %.0f deg)." % (ec * 57.3)); return

        self.log("==== SEQUENCE PRISE D'OUTIL (DLS) ====")
        self.lock(False)                                                          # 1
        if not self.straight(nid_p + np.array([0, 0, APPROACH]),
                             "montee nid +10cm", dry_start=nid): return            # 2
        if not self.free_to(list(appr_chg), "-> approche changeur",
                            dry_start=list(appr_nid)): return                      # 3
        self.grip(False)                                                          # 4
        if not self.straight(chg_p, "descente changeur",
                             dry_start=list(appr_chg)): return                     # 5
        self.lock(True)                                                           # 6
        if not self.straight(appr_chg_pos, "montee changeur +10cm",
                             dry_start=chg): return                                # 7
        self.grip(True); self.grip(False)                                         # 8
        self.log("==== FIN (a approche_changeur, pince prise) ====")

    def _save(self, name, joints):
        import re
        line = "%s: [%s]\n" % (name, ", ".join("%.4f" % v for v in joints))
        lines = []
        if os.path.exists(POSES_FILE):
            lines = [l for l in open(POSES_FILE) if not re.match(r"^%s\s*:" % re.escape(name), l)]
        lines.append(line); open(POSES_FILE, "w").write("".join(lines))
        self.log("  sauve %s" % name)


def main():
    dry = "--dry" in sys.argv
    rclpy.init(); n = Pickup(dry=dry)
    if dry:
        n.log("*** MODE DRY : validation, AUCUN mouvement ***")
    try:
        n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
