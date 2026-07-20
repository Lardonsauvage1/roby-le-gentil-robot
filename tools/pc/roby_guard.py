#!/usr/bin/env python3
"""
roby_guard.py — "Garde du corps" entre le reseau de neurones et les moteurs.

IDEE (modele mental de Sam) :
    RESEAU --consigne--> GARDE --> c'est OK ? --oui--> envoie aux moteurs
                                             --non--> STOP (gele sur place)

Le garde est le SEUL a parler au controleur des moteurs. Le reseau (ou le faux
reseau roby_replay.py) ne parle QU'AU garde, sur des topics /guard/... a part.
=> un seul proprietaire des moteurs, pas de "deux maitres qui se marchent dessus".

CE QU'IL VERIFIE, pour CHAQUE consigne, avant de la transmettre :
    1. BUTEES d'angle      -> reutilise O.LIMITS de roby_oracle (in-process, gratuit).
    2. VITESSE             -> |dq|/dt <= max-vel, et |dq| <= max-step (anti-saut violent).
    3. PLANCHER TABLE      -> FK de la consigne : la pince ne passe pas SOUS la table
                              (backup in-process, marche meme sans move_group).
    4. COLLISION reelle    -> MoveIt /check_state_validity (table + socle + auto-collision).
                              C'est la VRAIE anti-collision (scene chargee par scene_loader).
    Un seul test qui echoue => on NE transmet PAS.

CE QUE "STOP" VEUT DIRE (piege !) : PAS couper le courant (axes 2/3 tomberaient sous
la gravite). STOP = GELER = re-commander la position mesuree actuelle (les moteurs
TIENNENT), lever un drapeau d'echec, et ignorer le reseau jusqu'a un reset manuel.

DEUX ENTREES (memes verifications) :
    - STREAMING (vrai reseau)  : topic /guard/joint_trajectory (1..N points), a la volee.
    - BATCH (test replay)      : action /guard/follow_joint_trajectory (toute la traj d'un
                                 coup) -> validee entierement AVANT, puis transmise telle
                                 quelle a /arm_controller/follow_joint_trajectory.

TEST SANS VRAI RESEAU (recommande AVANT le reel) :
    # 1) stack up (controleurs) + move_group + scene :
    ros2 run roby_environments scene_loader --env atelier_actuel
    # 2) le garde :
    python3 ~/roby_guard.py
    # 3) le faux reseau, REMAPPE vers le garde (au lieu du controleur direct) :
    python3 ~/roby_replay.py <bag> --go \
        --ros-args -r /arm_controller/follow_joint_trajectory:=/guard/follow_joint_trajectory \
                   -r /gripper:=/guard/gripper
    # -> le bras rejoue l'episode A TRAVERS le garde. Injecter un bag "casse" pour
    #    verifier que le garde REFUSE (abort de l'action, aucun mouvement).

Reset apres un gel : ros2 service call /guard/reset std_srvs/srv/Trigger
Etat en direct    : ros2 topic echo /guard/status
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity

# --- cinematique + butees : une seule source de verite = roby_oracle.py ---
sys.path.insert(0, os.path.expanduser("~"))
import roby_oracle as O            # noqa: E402  (fkT, fk_pos, LIMITS, J, _z_pick, D_XYZ...)

J = O.J                            # ["joint_1"..."joint_5"]
NEPS = 1e-6


class Guard(Node):
    def __init__(self, a):
        super().__init__("roby_guard")
        self.a = a
        self.cb = ReentrantCallbackGroup()   # tout reentrant => on peut attendre un service dans un callback

        # --- etat ---
        self.cur = None            # position mesuree (np[5]) depuis /joint_states
        self.last_q = None         # derniere consigne TRANSMISE (pour la vitesse en streaming)
        self.last_t = None         # instant (monotonic) de cette derniere consigne
        self.frozen = False        # gele apres une violation ?
        self.reason = ""           # pourquoi
        self.n_pass = 0            # consignes transmises
        self.n_clamp = 0           # points clampes (vitesse/pas/butee limites)
        self.n_block = 0           # consignes/trajs refusees
        self.moveit_ready = False  # le service de collision a-t-il repondu au moins une fois ?

        # --- entrees / sorties ---
        self.sub_js = self.create_subscription(JointState, "/joint_states", self._on_js, 20, callback_group=self.cb)
        self.sub_tr = self.create_subscription(JointTrajectory, a.in_traj, self._on_nn_traj, 10, callback_group=self.cb)
        self.sub_gr = self.create_subscription(Bool, a.in_grip, self._on_nn_grip, 10, callback_group=self.cb)
        self.pub_tr = self.create_publisher(JointTrajectory, a.out_traj, 10)
        self.pub_gr = self.create_publisher(Bool, a.out_grip, 10)
        self.pub_st = self.create_publisher(String, "/guard/status", 10)

        # --- MoveIt : verificateur de collision (PAS un planificateur) ---
        self.moveit = self.create_client(GetStateValidity, "/check_state_validity", callback_group=self.cb)

        # --- reset manuel apres un gel ---
        self.srv_reset = self.create_service(Trigger, "/guard/reset", self._on_reset, callback_group=self.cb)

        # --- entree BATCH (pour tester avec roby_replay via l'action) ---
        self.act = ActionServer(
            self, FollowJointTrajectory, "/guard/follow_joint_trajectory",
            execute_callback=self._on_goal, goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda c: CancelResponse.ACCEPT, callback_group=self.cb)
        self.fwd = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory", callback_group=self.cb)

        self.create_timer(0.5, self._status, callback_group=self.cb)

        self.get_logger().info(
            f"GARDE actif. in={a.in_traj} -> out={a.out_traj} | max_vel={a.max_vel} rad/s "
            f"max_step={a.max_step} rad | moveit={'OUI' if not a.no_moveit else 'NON(degrade)'} "
            f"floor={'OUI' if not a.no_floor else 'NON'}")
        if a.no_moveit:
            self.get_logger().warn("--no-moveit : anti-collision reelle DESACTIVEE (seulement butees+vitesse+plancher).")

    # ===================== helpers etat =====================
    def _q_from_msg(self, names, positions):
        """Reordonne (names, positions) dans l'ordre J. None si un joint manque."""
        idx = {n: i for i, n in enumerate(names)}
        if not all(j in idx for j in J):
            return None
        return np.array([float(positions[idx[j]]) for j in J], float)

    def _on_js(self, m):
        q = self._q_from_msg(m.name, m.position)
        if q is not None:
            self.cur = q

    # ===================== les 4 verifications =====================
    def _check_limits(self, q):
        for k, jn in enumerate(J):
            lo, hi = O.LIMITS[jn]
            if not (lo - 0.02 <= q[k] <= hi + 0.02):
                return False, f"butee {jn}={q[k]:+.3f} hors [{lo:.2f},{hi:.2f}]"
        return True, ""

    def _check_speed(self, q, q_prev, dt):
        dq = np.abs(q - q_prev)
        # Saut absolu = anti-teleport, mais SEULEMENT sur un pas "instantane" (dt court). Un grand
        # deplacement etale sur un temps long (ex : lead-in 3 s vers la pose de depart) est legitime
        # -> il est juge par la vitesse ci-dessous, pas par ce cap.
        if dt < self.a.step_dt and float(dq.max()) > self.a.max_step:
            k = int(np.argmax(dq)); return False, f"saut {J[k]} d={dq[k]:.3f} > max_step {self.a.max_step} (dt={dt:.2f}s)"
        if dt > NEPS:
            v = dq / dt
            if float(v.max()) > self.a.max_vel:
                k = int(np.argmax(v)); return False, f"vitesse {J[k]} v={v[k]:.2f} > max_vel {self.a.max_vel} rad/s"
        return True, ""

    def _check_floor(self, q):
        if self.a.no_floor:
            return True, ""
        tcp = O.fk_pos(list(q))
        floor = O._z_pick(float(tcp[0]), float(tcp[1])) - self.a.floor_margin
        if float(tcp[2]) < floor:
            return False, f"plancher : pince z={tcp[2]:.3f} < table {floor:.3f}"
        return True, ""

    def _check_collision(self, q):
        """MoveIt : la config q est-elle en collision (table/socle/soi) ? Fail-safe."""
        if self.a.no_moveit:
            return True, ""
        if not self.moveit.service_is_ready():
            # service absent = on NE PEUT PAS garantir -> on REFUSE (fail-safe).
            return False, "MoveIt /check_state_validity absent (move_group + scene_loader lances ?)"
        req = GetStateValidity.Request()
        req.group_name = self.a.group
        req.robot_state.is_diff = True                 # nos 5 angles PAR-DESSUS l'etat courant de la scene
        req.robot_state.joint_state.name = list(J)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.moveit.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > self.a.moveit_timeout:
                return False, "MoveIt timeout"
            time.sleep(0.002)
        resp = fut.result()
        self.moveit_ready = True
        if resp is None:
            return False, "MoveIt pas de reponse"
        if not resp.valid:
            return False, "COLLISION (MoveIt)"
        return True, ""

    def _validate(self, q, q_prev, dt):
        """Les 4 tests, du moins cher au plus cher. (ok, raison)."""
        for chk in (self._check_limits(q), self._check_speed(q, q_prev, dt), self._check_floor(q)):
            if not chk[0]:
                return chk
        return self._check_collision(q)

    # ===================== chemin STREAMING (vrai reseau) =====================
    def _clamp(self, q, q_prev, dt):
        """RATE-LIMITER : limite le deplacement q_prev->q a max_vel (et max_step si dt court),
        puis aux butees. Retourne (q_clampe, a_ete_clampe). Le reseau garde son intention, la
        VITESSE est bornee (au lieu de geler)."""
        lim = self.a.max_vel * max(dt, 1e-3)              # deplacement max par la vitesse
        if dt < self.a.step_dt:
            lim = min(lim, self.a.max_step)               # + cap anti-teleport si pas instantane
        qc = q.copy(); clamped = False
        for j in range(len(J)):
            d = qc[j] - q_prev[j]
            if abs(d) > lim:
                qc[j] = q_prev[j] + (lim if d > 0 else -lim); clamped = True
        for k, jn in enumerate(J):                        # butees articulaires
            lo, hi = O.LIMITS[jn]
            if qc[k] < lo: qc[k] = lo; clamped = True
            elif qc[k] > hi: qc[k] = hi; clamped = True
        return qc, clamped

    def _on_nn_traj(self, msg):
        if self.frozen:
            return                                        # gele (collision) : on ignore jusqu'au reset
        if self.cur is None:
            self._freeze("pas encore de /joint_states (stack up ?)"); return
        out = JointTrajectory(); out.joint_names = list(J)
        q_prev = self.cur.copy()                          # on part de la position REELLE mesuree
        nclamp = 0
        for pt in msg.points:
            q = self._q_from_msg(msg.joint_names, pt.positions)
            if q is None:
                self._freeze("consigne : joints manquants / mauvais noms"); return
            tfs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = tfs if tfs > NEPS else 1.0 / max(self.a.assumed_rate, 1.0)
            qc, clamped = self._clamp(q, q_prev, dt)      # LIMITE vitesse/pas/butee (au lieu de geler)
            # collision/plancher : on ne peut pas clamper dans un obstacle -> GEL
            okc, why = self._check_floor(qc)
            if okc:
                okc, why = self._check_collision(qc)
            if not okc:
                self._freeze(why); return
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in qc]
            p.time_from_start = pt.time_from_start
            out.points.append(p)
            nclamp += int(clamped)
            q_prev = qc
        self.pub_tr.publish(out)                           # consigne clampee (sure)
        self.last_q = q_prev
        self.last_t = time.monotonic()
        self.n_pass += 1
        self.n_clamp += nclamp

    def _on_nn_grip(self, msg):
        if self.frozen:
            return                                        # pince aussi bloquee tant qu'on est gele
        self.pub_gr.publish(msg)                          # pince = peu dangereux -> passe telle quelle

    # ===================== gel / hold =====================
    def _freeze(self, reason):
        self.frozen = True
        self.reason = reason
        self.n_block += 1
        self.get_logger().error(f"!! STOP : {reason}  -> GEL (maintien position, moteurs tenus)")
        self._hold()

    def _hold(self):
        """Re-commande la position mesuree actuelle : les moteurs TIENNENT (pas de chute)."""
        if self.cur is None:
            return
        t = JointTrajectory(); t.joint_names = list(J)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in self.cur]
        p.time_from_start = Duration(sec=int(self.a.hold_time), nanosec=int((self.a.hold_time % 1) * 1e9))
        t.points.append(p)
        self.pub_tr.publish(t)

    def _on_reset(self, req, resp):
        self.frozen = False
        self.reason = ""
        self.last_q = None
        self.last_t = None
        self.get_logger().warn("RESET : le garde reprend (verifie la scene avant de relancer le reseau).")
        resp.success = True
        resp.message = "garde reactive"
        return resp

    # ===================== chemin BATCH (test replay via l'action) =====================
    def _on_goal(self, gh):
        """Valide TOUTE la trajectoire AVANT, puis la transmet au vrai controleur."""
        res = FollowJointTrajectory.Result()
        traj = gh.request.trajectory
        if self.cur is None:
            gh.abort(); res.error_string = "pas de /joint_states"; return res
        q_prev, t_prev = self.cur, 0.0
        for i, pt in enumerate(traj.points):
            q = self._q_from_msg(traj.joint_names, pt.positions)
            if q is None:
                gh.abort(); res.error_string = f"point {i}: joints manquants"; return res
            tfs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = max(tfs - t_prev, NEPS)
            ok, why = self._validate(q, q_prev, dt)
            if not ok:
                self.n_block += 1
                self.get_logger().error(f"!! Trajectoire REFUSEE au point {i}/{len(traj.points)} : {why}")
                gh.abort(); res.error_string = f"point {i}: {why}"; return res
            q_prev, t_prev = q, tfs
        # toute la traj est saine -> on la transmet au vrai controleur et on renvoie SON resultat
        if not self.fwd.wait_for_server(timeout_sec=5.0):
            gh.abort(); res.error_string = "arm_controller absent"; return res
        self.get_logger().info(f"Trajectoire OK ({len(traj.points)} points) -> transmise au controleur.")
        goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
        sf = self.fwd.send_goal_async(goal)
        while not sf.done():
            time.sleep(0.01)
        real = sf.result()
        if real is None or not real.accepted:
            gh.abort(); res.error_string = "controleur a refuse"; return res
        rf = real.get_result_async()
        while not rf.done():
            time.sleep(0.01)
        self.n_pass += 1
        gh.succeed()
        return rf.result().result

    # ===================== etat en direct =====================
    def _status(self):
        if self.frozen:
            s = f"FROZEN[{self.reason}] pass={self.n_pass} block={self.n_block}"
        else:
            mv = "moveit=off" if self.a.no_moveit else ("moveit=ok" if self.moveit_ready else "moveit=?")
            s = f"OK {mv} pass={self.n_pass} clamp={self.n_clamp} block={self.n_block}"
        self.pub_st.publish(String(data=s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", default="arm", help="groupe de planification MoveIt (SRDF)")
    # 1.0 rad/s = la limite declaree par l'URDF pour joint_1..3 (joint_4/5 sont a
    # 2.0, donc 1.0 est conservateur pour eux). Le defaut etait 1.5 : le garde,
    # cense etre le filtre le PLUS strict de la chaine, autorisait 50 % au-dessus
    # de la limite materielle declaree -- et 10x ce que s'imposent tous les autres
    # scripts (0.15). Sur un bras en boucle ouverte, depasser la vitesse declaree
    # = pas perdus (BUG-006). Corrige le 2026-07-20.
    ap.add_argument("--max-vel", type=float, default=1.0,
                    help="vitesse articulaire max (rad/s). Defaut = limite URDF joint_1..3. "
                         "Le garde CLAMPE (il ne gele pas) : une valeur plus basse ralentit, "
                         "elle n'interrompt pas.")
    ap.add_argument("--max-step", type=float, default=0.2, help="saut absolu max par point (rad, anti-teleport ; dataset réel: saut max 0.086/pas)")
    ap.add_argument("--step-dt", type=float, default=0.2, help="le cap max-step ne s'applique qu'aux pas de dt < step-dt (s)")
    ap.add_argument("--assumed-rate", type=float, default=30.0, help="cadence supposee si le point n'a pas de time_from_start")
    ap.add_argument("--floor-margin", type=float, default=0.03, help="marge sous la hauteur de prise avant 'sous la table' (m)")
    ap.add_argument("--hold-time", type=float, default=0.3, help="duree de la consigne de maintien au gel (s)")
    ap.add_argument("--moveit-timeout", type=float, default=0.5, help="timeout d'un appel check_state_validity (s)")
    ap.add_argument("--no-moveit", action="store_true", help="desactive l'anti-collision MoveIt (mode degrade)")
    ap.add_argument("--no-floor", action="store_true", help="desactive le plancher table FK (backup)")
    ap.add_argument("--in-traj", default="/guard/joint_trajectory")
    ap.add_argument("--in-grip", default="/guard/gripper")
    ap.add_argument("--out-traj", default="/arm_controller/joint_trajectory")
    ap.add_argument("--out-grip", default="/gripper")
    a = ap.parse_args()

    rclpy.init()
    node = Guard(a)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
