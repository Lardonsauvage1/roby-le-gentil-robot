#!/usr/bin/env python3
"""
roby_infer.py — Noeud d'inference (aligne sur le noeud de REFERENCE roby_infer_node.py de
l'equipe reseau). Diffusion Policy LeRobot, tache "prise de pomme".

DIFFERENCES CLES vs l'ancienne version (qui perdait des perfs) :
  - **select_action a 15 Hz** (la cadence d'entrainement, "NE PAS changer"), PAS 1 Hz ni de chunk maison.
  - archi async : un THREAD d'inference appelle select_action a 15 Hz -> tampon ; un TIMER 15 Hz publie
    1 point a la fois (tient le dernier si tampon vide). Decouple le calcul lourd (~540 ms) de la cadence.
  - select_action gere l'historique d'obs (n_obs_steps=2, espacement 1/15 s) et le chunking en INTERNE.

Sécurité : publie vers le GARDE (/guard/joint_trajectory + /guard/gripper), pas direct au controleur.
Le garde clampe (vitesse/collision) puis relaie. Gate arme/desarme via /roby_infer/arm (SetBool) pour
que la boucle oracle prenne/rende la main.

Pretraitement = EXACTEMENT le dataset : JPEG -> imdecode(BGR) -> resize 224 INTER_AREA -> BGR2RGB ->
CHW -> /255. La normalisation mean/std est faite par le `preprocessor` du checkpoint (pre/post OBLIGATOIRES).
Les images arrivent deja bien orientees (fix au niveau cam_pub/launch_cams) -> AUCUNE rotation ici.

CPU : epingler P-cores via roby_infer.sh (OMP_NUM_THREADS=6 taskset -c 0-11). torch.set_num_threads(6).
Usage : OMP_NUM_THREADS=6 taskset -c 0-11 python3 roby_infer.py --model <chemin> --go --arm-gate
"""
import argparse
import os
import threading
import time
from collections import deque

import torch
torch.set_num_threads(6)

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

# Pretraitement PARTAGE (roby_vision.py) : une seule implementation pour tous les
# noeuds d'inference. Evite qu'une copie divergE - notamment la resolution, qui etait
# figee a 224 en dur dans 3 fichiers et fausse sur les modeles 96/128.
from roby_vision import decode_resize, image_keys, img_size_from_policy
from roby_gripper import fermer as pince_fermer   # hysteresis : anti-claquement

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


class Infer(Node):
    def __init__(self, a):
        super().__init__("roby_infer")
        self.a = a
        self.hz = a.hz
        self.point_dt = 2.0 / self.hz               # time_from_start d'un setpoint (leger lissage)
        self.dev = torch.device("cpu")
        self.cb = ReentrantCallbackGroup()

        self.get_logger().info(f"chargement modele : {a.model}")
        self.policy = DiffusionPolicy.from_pretrained(a.model).eval().to(self.dev)
        self.policy.reset()
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.policy.config, pretrained_path=a.model,
            preprocessor_overrides={"device_processor": {"device": "cpu"}})
        self.img_keys = image_keys(self.policy)
        self.img_size = img_size_from_policy(self.policy, a.img_size)

        # GARDE-FOU dimensions (2026-07-20) : ce noeud suppose un modele JOINT,
        # c.-a-d. etat = 5 articulations et action = 5 articulations + pince.
        # Sans ce controle, passer un modele CARTESIEN par erreur (les deux .sh
        # acceptent --model) faisait envoyer a[:5] = [x, y, z, rvx, rvy] comme des
        # consignes ARTICULAIRES en radians. Le garde clampe, donc pas de casse,
        # mais le bras part n'importe ou. Symetrique du controle de roby_infer_cart.
        sdim = int(self.policy.config.input_features["observation.state"].shape[-1])
        adim = int(self.policy.config.output_features["action"].shape[-1])
        if (sdim, adim) != (5, 6):
            raise SystemExit(
                f"❌ modele state={sdim} action={adim} : ce n'est PAS un modele joint "
                f"(attendu 5/6). Pour un modele cartesien (6/7), utiliser roby_infer_cart.py.")
        # Pas de debruitage : --steps (defaut 10). Certains modeles sont entraines
        # pour MOINS (b2_perf_128 = DDIM 5 pas). Respecter la valeur d'entrainement.
        try:
            self.policy.diffusion.num_inference_steps = a.steps
            self._nsteps = a.steps
        except Exception as e:
            self._nsteps = None
            self.get_logger().warn(f"num_inference_steps non force: {e}")
        self.get_logger().info(
            f"img_size={self.img_size}  keys={self.img_keys}  num_inference_steps={self._nsteps}")

        # etat partage
        self.lock = threading.Lock()
        self.left = None; self.right = None; self.joints = None
        self.buf = deque(maxlen=64)                  # tampon d'actions (thread infer -> timer publish)
        self.last_action = None; self.last_grip = None
        self.armed = not a.arm_gate                  # --arm-gate : demarre DESARME (l'oracle arme)
        self.n_pub = 0; self.inf_ms = 0.0
        self.n_err = 0            # echecs CONSECUTIFS d'inference (remis a 0 au succes)
        self.n_err_total = 0      # cumul, pour le statut
        self.MAX_ERR = 5          # au-dela : desarmement automatique

        # --- entrees : cameras best-effort (comme cam_pub), joints ---
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, "/head_camera/left/image_raw/compressed",
                                 self._cl, qos_img, callback_group=self.cb)
        self.create_subscription(CompressedImage, "/head_camera/right/image_raw/compressed",
                                 self._cr, qos_img, callback_group=self.cb)
        self.create_subscription(JointState, "/joint_states", self._cj, 10, callback_group=self.cb)

        # --- sorties : vers le GARDE ---
        self.pub_traj = self.create_publisher(JointTrajectory, "/guard/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(Bool, "/guard/gripper", 10)
        self.pub_st = self.create_publisher(String, "/roby_infer/status", 10)
        self.create_service(Trigger, "/roby_infer/reset_episode", self._reset, callback_group=self.cb)
        self.create_service(SetBool, "/roby_infer/arm", self._arm, callback_group=self.cb)

        # --- thread infer + timers ---
        self.run = True
        threading.Thread(target=self._infer_loop, daemon=True).start()
        self.create_timer(1.0 / self.hz, self._publish_tick, callback_group=self.cb)
        self.create_timer(0.5, self._status, callback_group=self.cb)
        self.get_logger().info(
            f"pret. hz={self.hz} | images={self.img_keys} | "
            f"{'REEL -> /guard/*' if a.go else 'DRY (ne publie pas)'}"
            f"{' | arm-gate: DESARME' if a.arm_gate else ''}")

    # ---- callbacks : cache la derniere donnee de chaque flux ----
    def _cl(self, m):
        with self.lock: self.left = bytes(m.data)

    def _cr(self, m):
        with self.lock: self.right = bytes(m.data)

    def _cj(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(j in idx for j in J):
            q = np.array([float(m.position[idx[j]]) for j in J], np.float32)
            with self.lock: self.joints = q

    def _get_obs(self):
        two = len(self.img_keys) >= 2
        with self.lock:
            if self.left is None or self.joints is None or (two and self.right is None):
                return None
            l = self.left; r = self.right; q = self.joints.copy()
        # b2 MONO-camera : 1 seule cle image (observation.images.fixed) <- camera LEFT (fixe).
        # ancien 2-cam : img_keys[0] <- left, img_keys[1] <- right.
        obs = {self.img_keys[0]: decode_resize(l, self.dev, self.img_size),
               "observation.state": torch.from_numpy(q).unsqueeze(0).to(self.dev)}
        if two:
            obs[self.img_keys[1]] = decode_resize(r, self.dev, self.img_size)
        return obs

    # ---- thread d'inference : select_action a ~15 Hz, remplit le tampon (que si arme) ----
    def _infer_loop(self):
        """⚠️ Ce thread NE DOIT JAMAIS mourir en silence (fix 2026-07-20).

        Avant, aucun try/except : un seul JPEG corrompu (imdecode -> None, puis
        resize qui leve) tuait le thread DEFINITIVEMENT. Le timer de publication
        continuait alors a republier la derniere action indefiniment, le statut
        affichait toujours armed=True, et le bras tenait sa pose. Rien n'indiquait
        que le reseau ne pensait plus -- c'est le pire mode d'echec de la chaine,
        parce qu'il est indiscernable d'un modele qui a converge.
        """
        period = 1.0 / self.hz
        while self.run:
            if not self.armed:
                time.sleep(0.05); continue
            t0 = time.perf_counter()
            try:
                obs = self._get_obs()
                if obs is None:
                    time.sleep(0.05); continue
                with torch.no_grad():
                    a = self.post(self.policy.select_action(self.pre(obs)))   # pre/post OBLIGATOIRES
                a = a.squeeze(0).cpu().numpy()
                with self.lock:
                    self.buf.append(a)
                self.n_err = 0                          # succes : on repart de zero
            except Exception as e:
                self.n_err += 1
                self.n_err_total += 1
                self.get_logger().error(
                    f"inference EN ECHEC ({self.n_err}/{self.MAX_ERR}) : {type(e).__name__}: {e}")
                if self.n_err >= self.MAX_ERR:
                    # On DESARME plutot que de laisser le bras tenir une pose
                    # perimee en affichant "arme".
                    self.armed = False
                    with self.lock:
                        self.buf.clear()
                    self.get_logger().error(
                        f"{self.MAX_ERR} echecs consecutifs -> DESARMEMENT AUTOMATIQUE. "
                        f"Le reseau ne pilote plus. Reactiver via /roby_infer/arm apres diagnostic.")
                time.sleep(0.1)
                continue
            self.inf_ms = (time.perf_counter() - t0) * 1000
            dt = time.perf_counter() - t0
            if dt < period:                            # pacer a 15 Hz (les appels "file" sont rapides)
                time.sleep(period - dt)

    # ---- timer 15 Hz : publie 1 action depuis le tampon vers le garde ----
    def _publish_tick(self):
        if not (self.armed and self.a.go):
            return
        with self.lock:
            a = self.buf.popleft() if self.buf else self.last_action   # tient la derniere si vide
            if a is not None: self.last_action = a
        if a is None:
            return
        jt = JointTrajectory(); jt.joint_names = list(J)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in a[:5]]
        pt.time_from_start = Duration(sec=int(self.point_dt), nanosec=int((self.point_dt % 1) * 1e9))
        jt.points = [pt]
        self.pub_traj.publish(jt)                       # -> garde (clampe puis relaie)
        # HYSTERESIS : meme correctif que la version cartesienne (cf roby_gripper.py).
        close = pince_fermer(a[5], self.last_grip)
        if close != self.last_grip:
            self.pub_grip.publish(Bool(data=close))
            self.last_grip = close
            self.get_logger().info(f"  [pince] {'FERME' if close else 'OUVRE'}")
        self.n_pub += 1

    # ---- arme/desarme (boucle oracle) ----
    def _arm(self, req, resp):
        if req.data:
            self.policy.reset()
            with self.lock: self.buf.clear()
            self.last_action = None; self.last_grip = None
            self.armed = True
            self.get_logger().info("ARME : le reseau prend la main.")
        else:
            self.armed = False
            with self.lock: self.buf.clear()
            self.get_logger().info("DESARME.")
        resp.success = True; resp.message = f"armed={self.armed}"
        return resp

    def _reset(self, req, resp):
        self.policy.reset()
        with self.lock: self.buf.clear()
        self.last_action = None; self.last_grip = None
        self.get_logger().warn("RESET episode.")
        resp.success = True; resp.message = "reset ok"
        return resp

    def _status(self):
        # err= est expose EXPRES : c'est le seul moyen de distinguer "le modele a
        # converge et ne demande plus rien" de "le thread d'inference est mort".
        s = (f"hz={self.hz:.0f} armed={self.armed} inf={self.inf_ms:.0f}ms "
             f"pub={self.n_pub} buf={len(self.buf)} err={self.n_err_total} go={self.a.go}")
        self.pub_st.publish(String(data=s))

    def destroy_node(self):
        self.run = False
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="chemin checkpoint LeRobot (brut/ ou ema/)")
    ap.add_argument("--go", action="store_true", help="PUBLIE vers le garde (sinon DRY : infere sans publier)")
    ap.add_argument("--arm-gate", action="store_true", help="demarre DESARME : ne publie que si armé via /roby_infer/arm")
    ap.add_argument("--hz", type=float, default=15.0, help="cadence de controle = fps du DATASET d'entrainement (15 par defaut ; b2_perf_128 = 10)")
    ap.add_argument("--steps", type=int, default=10, help="pas de debruitage (10 ; b2_perf_128 DDIM = 5)")
    ap.add_argument("--img-size", type=int, default=0, help="resolution image (0=auto depuis la config du modele ; b2=96 ou 128)")
    a = ap.parse_args()
    a.model = os.path.expanduser(a.model)

    rclpy.init()
    node = Infer(a)
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
