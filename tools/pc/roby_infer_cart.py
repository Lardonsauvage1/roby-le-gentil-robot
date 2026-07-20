#!/usr/bin/env python3
"""
roby_infer_cart.py — Noeud d'inference pour le modele CARTESIEN (b2cart_fixed_*).
Copie de roby_infer.py, avec LES DEUX seules differences qui comptent :

  1. observation.state = POSE TCP 6D [x,y,z,rvx,rvy,rvz] (et non plus les 5 joints).
     Calculee ICI par FK des /joint_states avec fkT + rotvec = EXACTEMENT les fonctions
     de roby_dataset_to_cartesian.py qui a fabrique le dataset -> meme convention garantie.
     (Pas besoin d'un noeud /tcp_pose separe ; on le republie quand meme pour le debug.)
  2. action = 7 floats = [pose TCP cible 6D, pince]. Les 6 premiers sont convertis en
     joints par DLS (roby_tool_pickup.dls), seed = joints COURANTS -> chaque consigne
     repart de la pose reelle du bras.

⚠️ LATENCE : ce modele fait 263M params (U-Net 251M) => ~3.6 s par inference sur CPU,
pour 8 actions = 0.53 s de mouvement. Le bras BOUGE ~0.5 s puis TIENT ~3.5 s, en boucle.
C'est attendu et accepte (test de la chaine cartesienne, pas une demo de fluidite).
Quand le tampon est vide on republie la DERNIERE consigne => le bras tient sa position,
il ne retombe pas et ne saute pas (cf BUG-006 : arret/rattrapage violent).

Securite (en plus du garde) :
  - seed DLS = joints courants (jamais de reconstruction "dans le vide")
  - rejet si la FK ne recolle pas a la cible (FK_TOL)
  - clamp du deplacement TCP par pas (--max-dp) : borne les a-coups
  - rejet si saut articulaire > JUMP_TOL
  - publie vers /guard/* (le garde clampe vitesse/butees/collision puis relaie)

Usage : OMP_NUM_THREADS=6 taskset -c 0-11 python3 roby_infer_cart.py \
          --model ~/deployable_models/apple/b2cart_fixed_96/pretrained_model [--go] [--arm-gate]
        sans --go = DRY : infere et logue, NE PUBLIE RIEN.
"""
import argparse
import os
import sys
import threading
import time
from collections import deque

import torch
torch.set_num_threads(6)

import numpy as np
import cv2

sys.path.insert(0, os.path.expanduser("~"))
from roby_oracle import fkT, LIMITS                      # FK = celle du dataset
from roby_tool_pickup import dls, rotvec                 # IK amortie + rotation-vector
# Pretraitement PARTAGE : meme implementation que roby_infer.py (cf roby_vision.py).
# Le crop interne 84/112 est applique par LeRobot en eval() : ne PAS cropper ici.
from roby_vision import decode_resize, image_keys, img_size_from_policy
from roby_gripper import fermer as pince_fermer   # hysteresis : anti-claquement

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, String, Float32, Float64MultiArray
from std_srvs.srv import Trigger, SetBool

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

FK_TOL = 0.010      # m  : ecart max FK(joints reconstruits) vs pose TCP demandee
JUMP_TOL = 0.30     # rad: saut articulaire max autorise vs pose courante


def rv_to_R(rv):
    """rotation-vector -> matrice (Rodrigues). Inverse exact de rotvec()."""
    rv = np.asarray(rv, float)
    th = float(np.linalg.norm(rv))
    if th < 1e-9:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def tcp_of(q):
    """joints -> [x,y,z,rvx,rvy,rvz] : IDENTIQUE a roby_dataset_to_cartesian.py."""
    T = fkT(q)
    return np.concatenate([T[:3, 3], rotvec(T[:3, :3])])


class InferCart(Node):
    def __init__(self, a):
        super().__init__("roby_infer_cart")
        self.a = a
        self.hz = a.hz
        self.point_dt = 2.0 / self.hz
        self.dev = torch.device("cpu")
        self.cb = ReentrantCallbackGroup()

        self.get_logger().info(f"chargement modele CARTESIEN : {a.model}")
        self.policy = DiffusionPolicy.from_pretrained(a.model).eval().to(self.dev)
        self.policy.reset()
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.policy.config, pretrained_path=a.model,
            preprocessor_overrides={"device_processor": {"device": "cpu"}})
        self.img_keys = image_keys(self.policy)
        self.img_size = img_size_from_policy(self.policy, a.img_size)

        # GARDE-FOU : ce noeud suppose etat 6D + action 7D. Si le checkpoint ne colle pas,
        # c'est un modele JOINT -> refuser plutot que d'envoyer n'importe quoi au bras.
        sdim = int(self.policy.config.input_features["observation.state"].shape[-1])
        adim = int(self.policy.config.output_features["action"].shape[-1])
        if (sdim, adim) != (6, 7):
            raise SystemExit(
                f"❌ modele state={sdim} action={adim} : ce n'est PAS un modele cartesien "
                f"(attendu 6/7). Pour un modele joint (5/6), utiliser roby_infer.py.")

        self.policy.diffusion.num_inference_steps = a.steps
        self.get_logger().info(
            f"img_size={self.img_size} keys={self.img_keys} state={sdim} action={adim} "
            f"num_inference_steps={a.steps}")

        self.lock = threading.Lock()
        self.left = None
        self.joints = None
        self.buf = deque(maxlen=64)
        self.last_action = None
        self.last_grip = None
        self.armed = not a.arm_gate
        self.n_pub = 0
        self.n_reject = 0
        self.inf_ms = 0.0
        self.last_fk_mm = 0.0
        self.last_pred = None
        self.inf_heavy_ms = 0.0      # derniere VRAIE inference (pas un pop de file)
        self.n_err = 0               # echecs CONSECUTIFS d'inference
        self.n_err_total = 0
        self.MAX_ERR = 5             # au-dela : desarmement automatique

        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, "/head_camera/left/image_raw/compressed",
                                 self._cl, qos_img, callback_group=self.cb)
        self.create_subscription(JointState, "/joint_states", self._cj, 10, callback_group=self.cb)

        self.pub_traj = self.create_publisher(JointTrajectory, "/guard/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(Bool, "/guard/gripper", 10)
        self.pub_st = self.create_publisher(String, "/roby_infer/status", 10)
        self.pub_tcp = self.create_publisher(Float64MultiArray, "/tcp_pose", 10)   # debug/monitoring
        # DIAGNOSTIC pince : la valeur BRUTE continue predite (7e composante), AVANT seuillage.
        # C'est elle qui explique les claquements : si elle flotte autour de 0.5, l'etat bascule.
        # Publiee meme en DRY -> on peut observer sans que le bras bouge.
        self.pub_graw = self.create_publisher(Float32, "/roby_infer/gripper_raw", 10)
        self.pub_act = self.create_publisher(Float64MultiArray, "/roby_infer/action", 10)
        self.create_service(Trigger, "/roby_infer/reset_episode", self._reset, callback_group=self.cb)
        self.create_service(SetBool, "/roby_infer/arm", self._arm, callback_group=self.cb)

        self.run = True
        threading.Thread(target=self._infer_loop, daemon=True).start()
        self.create_timer(1.0 / self.hz, self._publish_tick, callback_group=self.cb)
        self.create_timer(0.5, self._status, callback_group=self.cb)
        self.get_logger().info(
            f"pret. hz={self.hz} | {'REEL -> /guard/*' if a.go else 'DRY (ne publie pas)'}"
            f"{' | arm-gate: DESARME' if a.arm_gate else ''}")
        # Avertissement calcule, PAS en dur : le gros modele (263M) et le petit (39M)
        # ont des comportements opposes et un texte fige mentirait sur l'un des deux.
        nparam = sum(p.numel() for p in self.policy.parameters())
        budget_s = self.policy.config.n_action_steps / self.hz
        if nparam > 1e8:
            self.get_logger().warn(
                f"modele LOURD ({nparam/1e6:.0f}M) : inference tres superieure au budget "
                f"de {budget_s:.2f}s -> le bras BOUGE par a-coups puis TIENT. C'est ATTENDU.")
        else:
            self.get_logger().info(
                f"modele leger ({nparam/1e6:.0f}M) : budget {budget_s:.2f}s pour "
                f"{self.policy.config.n_action_steps} actions -> mouvement quasi continu attendu.")

    def _cl(self, m):
        with self.lock:
            self.left = bytes(m.data)

    def _cj(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(j in idx for j in J):
            q = np.array([float(m.position[idx[j]]) for j in J], float)
            with self.lock:
                self.joints = q
            tcp = tcp_of(q)
            self.pub_tcp.publish(Float64MultiArray(data=[float(v) for v in tcp]))

    def _get_obs(self):
        with self.lock:
            if self.left is None or self.joints is None:
                return None
            l = self.left
            q = self.joints.copy()
        state = torch.from_numpy(tcp_of(q).astype(np.float32)).unsqueeze(0).to(self.dev)
        return {self.img_keys[0]: decode_resize(l, self.dev, self.img_size),
                "observation.state": state}

    def _infer_loop(self):
        period = 1.0 / self.hz
        while self.run:
            if not self.armed:
                time.sleep(0.05)
                continue
            t0 = time.perf_counter()
            try:
                obs = self._get_obs()
                if obs is None:
                    time.sleep(0.05)
                    continue
                with torch.no_grad():
                    a = self.post(self.policy.select_action(self.pre(obs)))
                a = a.squeeze(0).cpu().numpy()
                with self.lock:
                    self.buf.append(a)
                    self.last_pred = a
                self.n_err = 0
            except Exception as e:
                # Voir roby_infer.py : sans ce filet, un JPEG corrompu tuait le
                # thread et le bras tenait sa pose en affichant toujours "arme".
                self.n_err += 1
                self.n_err_total += 1
                self.get_logger().error(
                    f"inference EN ECHEC ({self.n_err}/{self.MAX_ERR}) : {type(e).__name__}: {e}")
                if self.n_err >= self.MAX_ERR:
                    self.armed = False
                    with self.lock:
                        self.buf.clear()
                    self.get_logger().error(
                        f"{self.MAX_ERR} echecs consecutifs -> DESARMEMENT AUTOMATIQUE.")
                time.sleep(0.1)
                continue
            self.inf_ms = (time.perf_counter() - t0) * 1000
            if self.inf_ms > 50:                 # >50ms = vraie inference, pas un pop de file
                self.inf_heavy_ms = self.inf_ms
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)

    def _publish_tick(self):
        if not self.armed:
            return
        with self.lock:
            fresh = bool(self.buf)                      # sinon = on TIENT la derniere consigne
            a = self.buf.popleft() if self.buf else self.last_action
            if a is not None:
                self.last_action = a
            q_cur = None if self.joints is None else self.joints.copy()
        if a is None or q_cur is None:
            return

        # --- diagnostic : toujours publie, meme en DRY (aucun mouvement induit) ---
        # signe de la valeur = drapeau "fraiche" : >=0 action NEUVE, <0 action TENUE
        # (le tampon se vide entre 2 inferences : ~85% des points sont des maintiens,
        #  les confondre donnerait une fausse lecture de ce que le modele DECIDE).
        self.pub_graw.publish(Float32(data=float(a[6]) if fresh else -float(a[6]) - 1e-6))
        self.pub_act.publish(Float64MultiArray(data=[float(x) for x in a]))

        if not self.a.go:
            return

        p_tgt = np.asarray(a[:3], float)
        R_tgt = rv_to_R(a[3:6])

        # --- clamp du pas cartesien : borne l'a-coup au redemarrage apres une pause ---
        p_cur = fkT(q_cur)[:3, 3]
        d = p_tgt - p_cur
        n = float(np.linalg.norm(d))
        if n > self.a.max_dp:
            p_tgt = p_cur + d / n * self.a.max_dp

        # --- IK amortie, seed = joints COURANTS (repart toujours du reel) ---
        j = np.asarray(dls(q_cur, p_tgt, R_tgt, iters=25, w_ori=self.a.w_ori), float)

        fk_err = float(np.linalg.norm(fkT(j)[:3, 3] - p_tgt))
        jump = float(np.max(np.abs(j - q_cur)))
        self.last_fk_mm = fk_err * 1000
        if fk_err > FK_TOL or jump > JUMP_TOL:
            self.n_reject += 1
            self.get_logger().warn(
                f"consigne REJETEE (FK {fk_err*1000:.1f}mm > {FK_TOL*1000:.0f} "
                f"ou saut {np.degrees(jump):.1f}deg > {np.degrees(JUMP_TOL):.0f}) -> maintien")
            return
        for k, name in enumerate(J):
            lo, hi = LIMITS[name]
            if not (lo <= j[k] <= hi):
                self.n_reject += 1
                self.get_logger().warn(f"{name}={j[k]:.3f} hors butee [{lo},{hi}] -> maintien")
                return

        jt = JointTrajectory()
        jt.joint_names = list(J)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in j]
        pt.time_from_start = Duration(sec=int(self.point_dt),
                                      nanosec=int((self.point_dt % 1) * 1e9))
        jt.points = [pt]
        self.pub_traj.publish(jt)

        # HYSTERESIS (2026-07-20) : un seuil unique a 0.5 faisait claquer la pince
        # toutes les ~65 ms quand la prediction flottait autour. Cf roby_gripper.py.
        close = pince_fermer(a[6], self.last_grip)
        if close != self.last_grip:
            self.pub_grip.publish(Bool(data=close))
            self.last_grip = close
            self.get_logger().info(f"  [pince] {'FERME' if close else 'OUVRE'}")
        self.n_pub += 1

    def _arm(self, req, resp):
        if req.data:
            self.policy.reset()
            with self.lock:
                self.buf.clear()
            self.last_action = None
            self.last_grip = None
            self.armed = True
            self.get_logger().info("ARME : le reseau cartesien prend la main.")
        else:
            self.armed = False
            with self.lock:
                self.buf.clear()
            self.get_logger().info("DESARME.")
        resp.success = True
        resp.message = f"armed={self.armed}"
        return resp

    def _reset(self, req, resp):
        self.policy.reset()
        with self.lock:
            self.buf.clear()
        self.last_action = None
        self.last_grip = None
        self.get_logger().warn("RESET episode.")
        resp.success = True
        resp.message = "reset ok"
        return resp

    def _status(self):
        s = (f"CART armed={self.armed} inf={self.inf_heavy_ms:.0f}ms pub={self.n_pub} "
             f"rejets={self.n_reject} buf={len(self.buf)} fk={self.last_fk_mm:.1f}mm "
             f"err={self.n_err_total} go={self.a.go}")
        self.pub_st.publish(String(data=s))
        # DIAGNOSTIC : ce que le reseau DEMANDE vs ou le bras EST. A lire avant d'autoriser --go.
        with self.lock:
            a = None if self.last_pred is None else self.last_pred.copy()
            q = None if self.joints is None else self.joints.copy()
        if a is None or q is None:
            return
        cur = tcp_of(q)
        d = np.asarray(a[:3], float) - cur[:3]
        self.get_logger().info(
            f"TCP reel [{cur[0]:+.3f} {cur[1]:+.3f} {cur[2]:+.3f}] -> demande "
            f"[{a[0]:+.3f} {a[1]:+.3f} {a[2]:+.3f}]  delta={np.linalg.norm(d)*1000:6.1f}mm "
            f"pince={'FERME' if a[6] > 0.5 else 'OUVRE'} inf={self.inf_heavy_ms:.0f}ms")

    def destroy_node(self):
        self.run = False
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="checkpoint cartesien (pretrained_model/)")
    ap.add_argument("--go", action="store_true", help="PUBLIE vers le garde (sinon DRY)")
    ap.add_argument("--arm-gate", action="store_true", help="demarre DESARME")
    ap.add_argument("--hz", type=float, default=15.0, help="cadence de publication (dataset = 15)")
    ap.add_argument("--img-size", type=int, default=0, help="0 = auto depuis la config")
    ap.add_argument("--steps", type=int, default=10, help="pas de debruitage (10 = doc ; moins = plus rapide, moins bon)")
    ap.add_argument("--max-dp", type=float, default=0.05, help="deplacement TCP max par consigne (m)")
    ap.add_argument("--w-ori", type=float, default=1.0, help="poids orientation du DLS (0.5 = priorite position)")
    a = ap.parse_args()
    a.model = os.path.expanduser(a.model)

    rclpy.init()
    node = InferCart(a)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass          # contexte deja ferme (SIGTERM) : rien a signaler


if __name__ == "__main__":
    main()
