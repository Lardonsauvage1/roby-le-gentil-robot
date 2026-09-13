"""Teleoperation du bras SIMULE par le joystick du bras guide (US-020).

Consomme `/leader/joystick` (consignes de VITESSE normalisees [-1, 1], deja exprimees
dans le repere de Roby) et les integre en positions articulaires, publiees sur
`/joint_states` pour l'affichage RViz.

    q(t+dt) = q(t) + v * vitesse_max * dt     puis CLAMP aux butees URDF

Aucun message n'est envoye au vrai robot : ce noeud ne fait qu'animer le modele. C'est
volontaire — on valide la chaine complete (guide -> conversion -> mouvement) sans risque
avant d'y brancher le vrai bras.

Pourquoi integrer plutot que mapper directement : le guide commande une vitesse, pas une
position. Relacher le guide (retour au neutre) ne doit PAS ramener Roby a une pose de
depart, mais l'arreter la ou il est.

    bash ~/roby_leader_teleop_sim.sh
"""

from __future__ import annotations

import math

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from roby_control.sim_joint_states import GardeJointStates

# Butees URDF de Roby (source : neuroneimitationcarote_description).
LIMITES = {
    "joint_1": (-3.14159, 3.14159),
    "joint_2": (-1.6, 2.1),
    "joint_3": (-3.0, 0.65),
    "joint_4": (-3.1416, 3.1416),
    "joint_5": (-1.6, 1.6),
}
# Pose de depart = centre de course (celle qui correspond au neutre du guide, US-019).
DEPART = {n: (lo + hi) / 2.0 for n, (lo, hi) in LIMITES.items()}


class TeleopSim(Node):
    def __init__(self):
        super().__init__("leader_teleop_sim")
        self.declare_parameter("vitesse_max_rad_s", 0.6)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("timeout_joystick_s", 0.5)
        # Gain de vitesse PAR AXE, dans l'ordre de LIMITES. Tous les axes n'ont pas
        # besoin de la meme sensibilite : la base et l'epaule deplacent beaucoup de bras
        # pour peu de degres, le poignet au contraire gagne a etre vif.
        self.declare_parameter("gains", [1.0, 1.0, 1.0, 1.0, 1.0])
        # Pose de depart, dans l'ordre de LIMITES. Vide = centre de course.
        # Sert notamment a placer le bras AU NID pour verifier l'alignement de la scene.
        # Valeur par defaut = centre de course, et non liste vide : d'une liste vide
        # rclpy deduit BYTE_ARRAY et refuse ensuite un tableau de reels.
        self.declare_parameter("pose_depart", [float(v) for v in DEPART.values()])
        # Verification anti-collision via MoveIt. Le mouvement candidat est teste AVANT
        # d'etre applique : si la pose visee est en collision, on ne bouge pas. La pose
        # courante restant valide, l'operateur se degage simplement en repoussant le
        # guide dans l'autre sens — on ne peut donc pas rester coince.
        self.declare_parameter("collision_check", True)
        self.declare_parameter("group_name", "arm")

        self.vmax = float(self.get_parameter("vitesse_max_rad_s").value)
        self.timeout = float(self.get_parameter("timeout_joystick_s").value)
        self.q = dict(DEPART)
        pd = list(self.get_parameter("pose_depart").value)
        if len(pd) != len(LIMITES):
            raise ValueError("pose_depart : %d valeurs pour %d articulations"
                             % (len(pd), len(LIMITES)))
        self.q = dict(zip(LIMITES.keys(), [float(v) for v in pd]))
        self.gains = dict(zip(LIMITES.keys(), list(self.get_parameter("gains").value)))
        self.cmd = {}
        self.dernier_msg = None

        self.cb = ReentrantCallbackGroup()
        self.collision = bool(self.get_parameter("collision_check").value)
        self.group = self.get_parameter("group_name").value
        self.cli = None
        self.bloque = False
        self.n_bloques = 0
        if self.collision:
            self.cli = self.create_client(GetStateValidity, "/check_state_validity",
                                          callback_group=self.cb)
            if self.cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info(
                    "Anti-collision ACTIF (MoveIt). La scene doit etre chargee : "
                    "ros2 run roby_environments scene_loader --env atelier_actuel")
            else:
                self.collision = False
                self.get_logger().error(
                    "/check_state_validity indisponible : ANTI-COLLISION DESACTIVE. "
                    "move_group tourne-t-il ? Le bras ne s'arretera PAS devant un obstacle.")

        self.create_subscription(JointState, "/leader/joystick", self._on_joy, 10,
                                 callback_group=self.cb)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        # Jamais a cote d'un vrai robot (BUG-008) : muet si un autre publisher existe.
        self._garde_js = GardeJointStates(self, self.pub)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick, callback_group=self.cb)
        self.add_on_set_parameters_callback(self._on_param)

        self.get_logger().info(
            "Teleop SIMULEE prete : %.2f rad/s a fond de course. Depart : %s. "
            "Aucun message vers le vrai robot."
            % (self.vmax, " ".join("%.3f" % v for v in self.q.values()))
        )

    def _on_joy(self, msg):
        self.cmd = dict(zip(msg.name, msg.velocity))
        self.dernier_msg = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        maintenant = self.get_clock().now().nanoseconds * 1e-9
        # Securite elementaire : sans consigne fraiche, on n'avance plus. Un guide
        # debranche ou un noeud mort ne doit pas laisser le bras filer sur sa derniere
        # consigne.
        frais = self.dernier_msg is not None and (maintenant - self.dernier_msg) < self.timeout
        if frais:
            cand = dict(self.q)
            bouge = False
            for nom, v in self.cmd.items():
                if nom not in cand:
                    continue
                lo, hi = LIMITES[nom]
                g = self.gains.get(nom, 1.0)
                nq = max(lo, min(hi, cand[nom] + v * g * self.vmax * self.dt))
                if nq != cand[nom]:
                    cand[nom], bouge = nq, True
            if bouge and self._pose_valide(cand):
                self.q = cand
            elif bouge:
                self.n_bloques += 1
                if not self.bloque:
                    self.bloque = True
                    self.get_logger().warn(
                        "COLLISION evitee : la pose demandee heurte un obstacle, le bras "
                        "est bloque dans cette direction. Repousser le guide dans l'autre "
                        "sens pour se degager.")
            if not bouge or self.q is cand:
                pass

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.q.keys())
        msg.position = [self.q[n] for n in msg.name]
        self._garde_js.publier(msg)

    def _pose_valide(self, q):
        """True si la pose est sans collision. En cas de doute -> False (on ne bouge pas)."""
        if not self.collision or self.cli is None:
            return True
        req = GetStateValidity.Request()
        req.group_name = self.group
        js = JointState()
        js.name = list(q.keys())
        js.position = [q[n] for n in js.name]
        rs = RobotState()
        rs.joint_state = js
        req.robot_state = rs
        fut = self.cli.call_async(req)
        # L'executeur est multi-thread : on peut attendre ici sans bloquer les autres
        # callbacks. Mesure du 2026-09-05 : 0,3 a 1,4 ms, tres en deca du cycle de 20 ms.
        for _ in range(40):
            if fut.done():
                break
            import time as _t
            _t.sleep(0.001)
        if not fut.done():
            self.get_logger().warn("verification de collision sans reponse : mouvement refuse")
            return False
        r = fut.result()
        valide = bool(r.valid) if r is not None else False
        if valide and self.bloque:
            self.bloque = False
            self.get_logger().info("degage : le mouvement reprend.")
        return valide

    def _on_param(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if p.name == "gains":
                vals = list(p.value)
                if len(vals) != len(LIMITES):
                    return SetParametersResult(successful=False,
                                               reason="il faut %d gains" % len(LIMITES))
                self.gains = dict(zip(LIMITES.keys(), [float(v) for v in vals]))
                self.get_logger().warn("gains -> %s" % ", ".join(
                    "%s x%.2f" % (n, g) for n, g in self.gains.items()))
                continue
            if p.name == "vitesse_max_rad_s":
                if not 0.0 < float(p.value) <= 3.0:
                    return SetParametersResult(successful=False,
                                               reason="vitesse hors de ]0, 3] rad/s")
                self.vmax = float(p.value)
                self.get_logger().warn("vitesse max -> %.2f rad/s (%.0f deg/s)"
                                       % (self.vmax, math.degrees(self.vmax)))
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
