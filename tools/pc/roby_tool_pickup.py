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

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))  # voisins de CE fichier, pas ceux du home

import roby_gates
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
# --- Compensation TEMPORAIRE de l'echelle de joint_3 (2026-09-07) ---
# DESACTIVEE PAR DEFAUT. Activer par : export ROBY_J3_SCALE=0.9299
# 0.9299 = fraction de la course commandee que joint_3 parcourt reellement,
# deduite de deux mesures pince/table (voir NOTES_echelle_joint3.md).
# Rien n'est ecrit en dur : au prochain demarrage sans la variable, le
# comportement d'origine revient.
# Pause immobile observee AVANT chaque ouverture/fermeture de pince (s).
GRIP_SETTLE = float(os.environ.get("ROBY_GRIP_SETTLE", "0.5"))
# Tenue supplementaire APRES une OUVERTURE (= lacher l'objet), avant de bouger.
# Constat Sam 2026-09-07 : la pomme etait lachee ~2 s apres l'ordre, donc EN PLEIN
# mouvement de remontee. Les 1.5 s d'attente du grip() ne suffisaient pas a couvrir
# la latence servo. On tient donc 1 s de plus, bras immobile, avant de repartir --
# sinon l'objet est traine et l'instant du lacher est faux dans le bag.
GRIP_RELEASE_HOLD = float(os.environ.get("ROBY_GRIP_RELEASE_HOLD", "1.0"))

J3_SCALE = float(os.environ.get("ROBY_J3_SCALE", "0") or 0)
J3_REF = float(os.environ.get("ROBY_J3_REF", "0.5237"))   # joint_3 au nid = ancre

FREE_VEL = 0.50            # facteur vitesse du mouvement libre MoveIt (x5 vs le run lent 0.10)
DESCENT_W_ORI = 1.0        # poids orientation des LIGNES DROITES (1=strict, changeur d'outil).
                           # L'oracle le baisse (~0.2) pour prioriser la position sur toute la table.
POSES_FILE = os.path.expanduser("~/roby_poses.yaml")


# ---------- FK / Jacobienne (repère link_gripper) : LUES DANS L'URDF ----------
# ADR-005 (2026-09-20) : la geometrie du bras n'a qu'une source, l'URDF du depot. Elle
# n'est plus recopiee ici. Les noms restent exportes pour les scripts qui les importent
# depuis ce module (roby_replay_cartesian, roby_infer_cart, roby_oracle...).
from roby_cinematique import (  # noqa: E402
    Rz, Ry, Rx, H, fkT, fk_pos, fk_poignet, rotvec, jac, LIMITS, CHEMIN_URDF,
)


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
        if not dry:
            # Porte : libre par MoveIt (scene exigee), lignes droites directes au controleur
            # (/joint_states sans fantome exige). L'oracle passe par ici.
            roby_gates.exiger(self, "roby_tool_pickup", scene=True)
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

    def _decompense(self, joints):
        """Inverse de _compense : d'une valeur LUE sur /joint_states vers l'espace MODELE.

        En boucle ouverte /joint_states renvoie la CONSIGNE, donc une valeur deja
        compensee. La position physique reelle vaut J3_REF + J3_SCALE*(lu - J3_REF) :
        de-compenser, c'est donc rendre la vraie position du bras, pas une convention.
        """
        if J3_SCALE <= 0 or joints is None:
            return joints if joints is None else list(joints)
        out = list(joints)
        out[2] = J3_REF + (out[2] - J3_REF) * J3_SCALE
        return out

    def wait_state(self):
        """Position courante en espace MODELE (de-compensee).

        ⚠️ BUG VECU (2026-09-07) : sans la de-compensation, straight() partait d'une
        valeur deja compensee, la traitait comme du modele, puis _exec_traj la
        recompensait -- soit ~74 mrad de saut sur joint_3 AU PREMIER POINT de chaque
        ligne droite. Symptomes rapportes par Sam : une secousse vers le HAUT juste
        avant chaque descente, et un depart tres rapide a chaque remontee.
        La regle : espace modele a l'interieur, compensation seulement a l'envoi.
        """
        t0 = time.time()
        while self.cur is None and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._decompense(self.cur)

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
                # marge A L'INTERIEUR (cf. _in_limits de roby_oracle) : finir une ligne
                # droite sur la butee bloque MoveIt pour tout le reste de la session.
                lo, hi = LIMITS[jn]
                if not (lo + 0.02 <= j[k] <= hi - 0.02):
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

    def _compense(self, joints):
        """Compensation TEMPORAIRE de l'echelle de joint_3 (mesures 2026-09-07).

        OPT-IN : sans la variable d'environnement ROBY_J3_SCALE, cette fonction ne
        fait RIEN. Un redemarrage sans la variable rend donc le comportement
        d'origine -- c'est voulu : la vraie reparation est dans la config du driver
        (joint_3_gear_ratio_*), pas ici. Voir NOTES_echelle_joint3.md.

        Mesure : joint_3 ne parcourt qu'une fraction de la course commandee. On
        pre-deforme donc la consigne autour de sa valeur au nid (l'ancre de la
        boucle ouverte, ou l'erreur est nulle par construction).

        ⚠️ Effet de bord a connaitre pour le dataset : /joint_states renverra la
        consigne COMPENSEE. Une FK appliquee telle quelle donnera un cartesien FAUX.
        Il faut d'abord defaire la compensation (cf. le fichier de notes).
        """
        if J3_SCALE <= 0:
            return list(joints)
        out = list(joints)
        out[2] = J3_REF + (out[2] - J3_REF) / J3_SCALE
        return out

    def _exec_traj(self, wps, total):
        if not self.traj_ac.wait_for_server(timeout_sec=10):
            self.log("  arm_controller absent"); return False
        traj = JointTrajectory(); traj.joint_names = list(J)
        for i, wp in enumerate(wps):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in self._compense(wp)]
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
        for jn, val in zip(J, self._compense(joints)):
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
        # PAUSE IMMOBILE AVANT d'actionner (demande Sam 2026-09-07) : le bras finit de
        # s'immobiliser avant que les doigts bougent. Sans elle, la pince s'actionne
        # pendant que la trajectoire finit de s'amortir -- la pomme peut etre poussee, et
        # l'instant de la saisie est mal defini dans le bag (le reseau apprend un geste flou).
        self._spin(GRIP_SETTLE if not self.dry else 0.0)
        if not self.dry:
            self.grip_pub.publish(Bool(data=close))
        self.log("  pince %s%s" % ("FERME" if close else "OUVRE", " [DRY]" if self.dry else ""))
        self._spin(0.3 if self.dry else 1.5)
        if not close and not self.dry and GRIP_RELEASE_HOLD > 0:
            # lacher : on laisse l'objet se poser AVANT tout mouvement
            self._spin(GRIP_RELEASE_HOLD)

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
