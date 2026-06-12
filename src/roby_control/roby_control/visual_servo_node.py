#!/usr/bin/env python3
"""
visual_servo_node — pilote le TCP du bras simulé pour suivre le cube ArUco.

Conçu (2026-06-12) après le constat que MoveIt Servo est inutilisable en cartésien
sur un bras 5-DOF (cf mémoire project_moveit_servo_5dof_limite). On fait donc notre
propre IK différentielle à moindres carrés amortis (DLS) :

    q̇ = J_selᵀ · (J_sel·J_selᵀ + λ²·I)⁻¹ · ẋ_sel

avec sélection explicite des DOF de tâche (J_sel = lignes choisies du Jacobien),
ce qui évite la sur-contrainte d'un 5-DOF. λ amortit près des singularités
(pas de faux arrêt, pas d'à-coup). Intégration depuis la pose courante → continuité
garantie (pas de saut de branche IK).

Pipeline :
  /cube_pose → EMA lissage → remap échelle+recentrage (design "téléop par cube")
            → cible TCP bornée → erreur → DLS → check collision → limites → /arm_controller/joint_trajectory

Suivi de POSITION par défaut (3 DOF). Orientation (option 3 "la pince imite le cube")
câblée mais OFF par défaut : elle dépend de la calibration des faces du cube + d'un
repère caméra→base, à faire plus tard. Activer via param track_orientation.

Sécurité (4 couches) : butées articulaires, boîte de travail bornée, limites
vitesse, et check collision via le service /check_state_validity de move_group.

Services : /visual_servo/enable (std_srvs/SetBool) — démarre/arrête le suivi.
           À l'activation : capture la pose cube de référence + le home TCP courant.
"""

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

# --- Cinématique du bras (URDF, tous rpy=0) — identique au prototype validé ---
def Rx(q): c, s = np.cos(q), np.sin(q); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(q): c, s = np.cos(q), np.sin(q); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(q): c, s = np.cos(q), np.sin(q); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

JOINTS = [
    (np.array([0, 0, 0.02]),            np.array([0, 0, 1.]), Rz),
    (np.array([0.024031, 0, 0.202992]), np.array([0, 1, 0.]), Ry),
    (np.array([-0.015224, 0, 0.441653]),np.array([0, 1, 0.]), Ry),
    (np.array([0.119473, 0, 0.029716]), np.array([1, 0, 0.]), Rx),
    (np.array([0.321516, 0, 0]),        np.array([0, 1, 0.]), Ry),
]
TCP_OFF = np.array([0.16, 0, 0])  # link_5 -> tcp (0.06 gripper + 0.10 tcp)
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
Q_LO = np.array([-3.14159, -1.0, -3.0, -3.14159, -1.6])
Q_HI = np.array([3.14159, 1.2, 0.65, 3.14159, 1.6])


def quat_to_R(x, y, z, w):
    n = np.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def rotvec_from_R(R):
    """Matrice de rotation -> vecteur rotation (axe·angle), en rad."""
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    angle = np.arccos(c)
    if angle < 1e-6:
        return np.zeros(3)
    axis = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    s = np.linalg.norm(axis)
    if s < 1e-9:   # angle ~ pi : axe via diagonale
        k = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        axis = R[:, k] + np.eye(3)[:, k]
        return axis / (np.linalg.norm(axis) or 1.0) * angle
    return axis / s * angle


def fk_jac(q):
    """Retourne (position TCP, rotation TCP 3x3, Jacobien géométrique 6x5)."""
    R = np.eye(3); p = np.zeros(3); zs = []; ps = []
    for i, (o, ax, rf) in enumerate(JOINTS):
        p = p + R @ o
        zs.append(R @ ax); ps.append(p.copy())
        R = R @ rf(q[i])
    p_tcp = p + R @ TCP_OFF
    J = np.zeros((6, 5))
    for i in range(5):
        J[:3, i] = np.cross(zs[i], p_tcp - ps[i])
        J[3:, i] = zs[i]
    return p_tcp, R, J


class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        # --- paramètres ---
        p = self.declare_parameter
        p('control_rate', 30.0)
        p('ema_alpha', 0.5)            # lissage cube (0..1 ; grand = + réactif/moins lissé)
        p('scale', 1.5)               # gain de remap : déplacement cube -> TCP
        p('kp', 4.0)                  # gain proportionnel de la boucle TCP (réactivité)
        p('max_linear_speed', 0.35)   # m/s
        p('lambda_dls', 0.1)          # amortissement DLS
        p('max_joint_speed', 1.5)     # rad/s
        p('max_disp', 0.25)           # déplacement TCP max autour du home (m)
        p('dead_zone', 0.005)         # m, en dessous on ne bouge pas
        p('cube_timeout', 2.5)        # tolère les décrochages pendant la rotation du cube
        p('track_orientation', True)   # option 3 : la pince imite l'orientation du cube
        p('korient', 3.0)              # gain orientation (rad/s par rad d'erreur)
        p('max_angular_speed', 2.0)    # rad/s
        p('enable_collision_check', True)
        # boîte de travail absolue (repère base_link) — filet de sécurité
        p('ws_min', [-0.1, -0.6, 0.05])
        p('ws_max', [0.80, 0.6, 0.95])
        # mapping déplacement caméra -> base (matrice 3x3 aplatie, éditable si le
        # "sens" du suivi est inversé à l'écran). Défaut : cam optique (x droite,
        # y bas, z avant) -> base (x avant, y gauche, z haut).
        p('cam_to_base', [0., 0., 1.,  -1., 0., 0.,  0., -1., 0.])

        g = lambda n: self.get_parameter(n).value
        self.alpha = g('ema_alpha'); self.scale = g('scale'); self.kp = g('kp')
        self.vmax = g('max_linear_speed'); self.lam = g('lambda_dls')
        self.qd_max = g('max_joint_speed'); self.max_disp = g('max_disp')
        self.dead = g('dead_zone'); self.cube_timeout = g('cube_timeout')
        self.track_ori = g('track_orientation')
        self.korient = g('korient'); self.wmax = g('max_angular_speed')
        self.collision_check = g('enable_collision_check')
        self.ws_min = np.array(g('ws_min')); self.ws_max = np.array(g('ws_max'))
        self.M = np.array(g('cam_to_base')).reshape(3, 3)
        # masque DOF de tâche : position (3) + orientation (3) si activée
        self.mask = np.array([1, 1, 1, 1, 1, 1] if self.track_ori else [1, 1, 1, 0, 0, 0], bool)

        # --- état ---
        self.q = None                 # joints courants
        self.cube_raw = None          # dernière position cube (repère caméra)
        self.cube_ema = None          # position cube lissée
        self.cube_R = None            # dernière orientation cube (repère caméra)
        self.last_cube_t = None
        self.active = False
        self.cube_ref = None          # position cube de référence (à l'activation)
        self.cube_ref_R = None        # orientation cube de référence (à l'activation)
        self.tcp_home = None          # position TCP de référence (à l'activation)
        self.tcp_home_R = None        # orientation TCP de référence (à l'activation)
        self.candidate = None         # dernier q_next (pour check collision async)
        self.collision_ok = True

        # --- ROS I/O ---
        cg = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 10, callback_group=cg)
        self.create_subscription(PoseStamped, '/cube_pose', self.cb_cube, 10, callback_group=cg)
        self.cmd_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(String, '/visual_servo/status', 10)
        self.srv = self.create_service(SetBool, '/visual_servo/enable', self.cb_enable, callback_group=cg)

        self.col_client = self.create_client(GetStateValidity, '/check_state_validity', callback_group=cg)
        self.dt = 1.0 / g('control_rate')
        self.create_timer(self.dt, self.control_loop, callback_group=cg)

        self.get_logger().info(
            f"visual_servo prêt. Orientation={'ON' if self.track_ori else 'OFF'}, "
            f"collision_check={'ON' if self.collision_check else 'OFF'}. "
            f"Appeler /visual_servo/enable data:true pour démarrer.")

    # ---------- callbacks entrées ----------
    def cb_joints(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in JOINT_NAMES):
            self.q = np.array([d[n] for n in JOINT_NAMES])

    def cb_cube(self, m):
        pos = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.cube_raw = pos
        self.cube_ema = pos if self.cube_ema is None else (
            self.alpha * pos + (1 - self.alpha) * self.cube_ema)
        o = m.pose.orientation
        self.cube_R = quat_to_R(o.x, o.y, o.z, o.w)
        self.last_cube_t = self.get_clock().now()

    def cb_enable(self, req, resp):
        if req.data:
            if self.q is None:
                resp.success = False; resp.message = "pas d'état articulaire"; return resp
            if self.cube_ema is None:
                resp.success = False; resp.message = "cube non détecté (montre-le à la caméra)"; return resp
            # capture des références : centre cube + home TCP (position ET orientation)
            self.cube_ref = self.cube_ema.copy()
            self.cube_ref_R = self.cube_R.copy() if self.cube_R is not None else np.eye(3)
            self.tcp_home, R_home, _ = fk_jac(self.q)
            self.tcp_home_R = R_home
            self.active = True
            resp.success = True; resp.message = "suivi démarré"
            self.get_logger().info(f"Suivi ON. home TCP={np.round(self.tcp_home,3)}")
        else:
            self.active = False
            resp.success = True; resp.message = "suivi arrêté"
            self.get_logger().info("Suivi OFF.")
        return resp

    # ---------- check collision (synchrone, avec recul progressif) ----------
    def _state_valid(self, q):
        """True si la config q est sans collision (via /check_state_validity)."""
        if not self.col_client.service_is_ready():
            return True   # service absent -> ne pas bloquer
        req = GetStateValidity.Request()
        rs = RobotState()
        rs.joint_state.name = JOINT_NAMES
        rs.joint_state.position = [float(x) for x in q]
        req.robot_state = rs
        req.group_name = 'arm'
        ev = threading.Event(); box = {'v': True}
        fut = self.col_client.call_async(req)

        def done(f):
            try:
                box['v'] = bool(f.result().valid)
            except Exception:
                box['v'] = True
            ev.set()
        fut.add_done_callback(done)
        ev.wait(timeout=0.04)
        return box['v']

    def safe_step(self, q_next):
        """Plus grande fraction du pas q->q_next qui reste sans collision.
        Le bras avance jusqu'au mur de collision au lieu de se figer."""
        if not self.collision_check:
            return q_next, True
        if self._state_valid(q_next):
            return q_next, True
        dq = q_next - self.q
        lo, hi = 0.0, 1.0
        for _ in range(4):              # recherche dichotomique de la fraction sûre
            mid = (lo + hi) / 2.0
            if self._state_valid(self.q + mid * dq):
                lo = mid
            else:
                hi = mid
        return self.q + lo * dq, lo > 0.02

    # ---------- boucle de contrôle ----------
    def set_status(self, s):
        self.status_pub.publish(String(data=s))

    def control_loop(self):
        if not self.active or self.q is None:
            self.set_status('disabled'); return
        # cube perdu ?
        if self.last_cube_t is None or \
           (self.get_clock().now() - self.last_cube_t).nanoseconds > self.cube_timeout * 1e9:
            self.set_status('lost'); return

        # 1) cible TCP = home + remap(déplacement cube) borné
        d_cam = self.cube_ema - self.cube_ref
        d_base = self.M @ d_cam * self.scale
        n = np.linalg.norm(d_base)
        if n > self.max_disp:
            d_base = d_base / n * self.max_disp
        target = np.clip(self.tcp_home + d_base, self.ws_min, self.ws_max)

        # 2) erreur position TCP courant
        p_tcp, R_tcp, J = fk_jac(self.q)
        err = target - p_tcp
        if np.linalg.norm(err) < self.dead:
            v = np.zeros(3)                  # zone morte position (mais orientation continue)
        else:
            v = self.kp * err
            vn = np.linalg.norm(v)
            if vn > self.vmax:
                v = v / vn * self.vmax

        # 3) vitesse angulaire désirée (orientation : la pince imite le cube)
        w = np.zeros(3)
        do_ori = self.track_ori and self.cube_R is not None and self.cube_ref_R is not None
        if do_ori:
            dR_cam = self.cube_R @ self.cube_ref_R.T   # rotation cube depuis réf (caméra)
            dR_base = self.M @ dR_cam @ self.M.T        # ...ramenée en repère base
            R_des = dR_base @ self.tcp_home_R           # orientation pince visée
            R_err = R_des @ R_tcp.T                      # erreur (base)
            w = self.korient * rotvec_from_R(R_err)
            wn = np.linalg.norm(w)
            if wn > self.wmax:
                w = w / wn * self.wmax

        # 4) IK PRIORITAIRE : position = tâche primaire (suivie exactement si
        #    atteignable), orientation = tâche secondaire dans l'ESPACE NUL de la
        #    position (au mieux avec les 2 axes restants). => priorité xyz.
        lam2 = self.lam ** 2
        Jp = J[:3]                                       # Jacobien position 3x5
        Jp_pinv = Jp.T @ np.linalg.inv(Jp @ Jp.T + lam2 * np.eye(3))
        qdot = Jp_pinv @ v                               # tâche primaire = position
        if do_ori:
            Jo = J[3:]                                   # Jacobien orientation 3x5
            N = np.eye(5) - Jp_pinv @ Jp                 # projecteur espace nul position
            JoN = Jo @ N
            JoN_pinv = JoN.T @ np.linalg.inv(JoN @ JoN.T + lam2 * np.eye(3))
            qdot = qdot + JoN_pinv @ (w - Jo @ qdot)     # orientation sans gêner la position

        # 5) limites vitesse articulaire + intégration + butées
        qdot = np.clip(qdot, -self.qd_max, self.qd_max)
        q_next = np.clip(self.q + qdot * self.dt, Q_LO, Q_HI)

        # 6) recul progressif si collision (avance jusqu'au mur, ne fige pas)
        q_cmd, moved = self.safe_step(q_next)

        # 7) publication vers le contrôleur
        jt = JointTrajectory(); jt.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_cmd]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(self.dt * 1e9)
        jt.points = [pt]
        self.cmd_pub.publish(jt)
        self.set_status('tracking' if moved else 'limit')


def main():
    rclpy.init()
    node = VisualServoNode()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
