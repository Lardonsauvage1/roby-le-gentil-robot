#!/usr/bin/env python3
"""
roby_infer_bag.py — Nourrit le RESEAU avec un enregistrement du DATASET (cams + joints) au lieu du
live, pour voir son comportement sur une entree PARFAITEMENT in-distribution. La sortie du reseau
pilote le vrai robot VIA LE GARDE (qui clampe/protege).

But : isoler le MODELE des soucis de scene/camera live. Si le reseau reproduit bien l'episode
enregistre (le bras refait le geste), le modele est bon et le probleme etait l'entree live.

Chaine : bag (obs) -> select_action (15 Hz) -> /guard/joint_trajectory + /guard/gripper -> garde -> robot.

Usage : OMP_NUM_THREADS=6 taskset -c 0-11 python3 roby_infer_bag.py --model <chemin> --bag <ep_dir> [--go]
  sans --go : DRY (infere + log, ne publie pas). avec --go : pilote le robot via le garde.
"""
import argparse
import os
import time

import torch
torch.set_num_threads(6)
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, CompressedImage
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

# Pretraitement PARTAGE : 4e et derniere copie de decode_resize eliminee (2026-07-20).
# Ce fichier figeait la resolution a 224 en dur, ce qui le rendait FAUX sur tous les
# modeles actuels (96/128) -- le risque n'etait pas la formule, identique partout,
# mais le parametre.
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.expanduser("~"))
from roby_vision import decode_resize, image_keys, img_size_from_policy

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def read_bag_obs(bag, hz):
    """Lit le bag -> liste d'obs re-echantillonnee a hz : [(left_jpeg, right_jpeg, joints[5]), ...]."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    js, left, right = [], [], []
    while r.has_next():
        topic, data, t = r.read_next(); ts = t / 1e9
        if topic == "/joint_states":
            m = deserialize_message(data, JointState); idx = {n: i for i, n in enumerate(m.name)}
            if all(j in idx for j in J):
                js.append((ts, [float(m.position[idx[j]]) for j in J]))
        elif topic.endswith("left/image_raw/compressed"):
            m = deserialize_message(data, CompressedImage); left.append((ts, bytes(m.data)))
        elif topic.endswith("right/image_raw/compressed"):
            m = deserialize_message(data, CompressedImage); right.append((ts, bytes(m.data)))
    if not js or not left or not right:
        raise RuntimeError(f"bag incomplet : joints={len(js)} left={len(left)} right={len(right)}")
    t0, tN = js[0][0], js[-1][0]
    def near(seq, t): return min(seq, key=lambda e: abs(e[0] - t))[1]
    obs = []
    for i in range(int((tN - t0) * hz)):
        t = t0 + i / hz
        obs.append((near(left, t), near(right, t), np.array(near(js, t), np.float32)))
    return obs


class InferBag(Node):
    def __init__(self, a):
        super().__init__("roby_infer_bag")
        self.a = a; self.dev = torch.device("cpu")
        self.get_logger().info(f"chargement modele : {a.model}")
        self.policy = DiffusionPolicy.from_pretrained(a.model).eval().to(self.dev)
        self.policy.reset()
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.policy.config, pretrained_path=a.model,
            preprocessor_overrides={"device_processor": {"device": "cpu"}})
        self.img_keys = image_keys(self.policy)
        self.img_size = img_size_from_policy(self.policy)
        self.point_dt = 2.0 / a.hz
        self.pub_traj = self.create_publisher(JointTrajectory, "/guard/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(Bool, "/guard/gripper", 10)
        self.get_logger().info(f"lecture bag : {a.bag}")
        self.obs = read_bag_obs(a.bag, a.hz)
        self.get_logger().info(f"{len(self.obs)} pas @ {a.hz:.0f}Hz ({len(self.obs)/a.hz:.1f}s) | {'REEL->/guard' if a.go else 'DRY'}")

    def run(self):
        period = 1.0 / self.a.hz
        # ===== PHASE 1 : PRE-CALCUL de toutes les actions (le robot NE BOUGE PAS) =====
        # (l'inference lourde est bursty ; on la sort de la boucle de publication pour eviter les saccades)
        self.policy.reset()
        self.get_logger().info("PHASE 1 : precalcul des actions (aucun mouvement)...")
        acts = []
        for i, (l, r, q) in enumerate(self.obs):
            # mono-camera (modeles b2*) : une seule cle. 2 cameras : ancienne generation.
            obs = {self.img_keys[0]: decode_resize(l, self.dev, self.img_size),
                   "observation.state": torch.from_numpy(q).unsqueeze(0).to(self.dev)}
            if len(self.img_keys) > 1:
                obs[self.img_keys[1]] = decode_resize(r, self.dev, self.img_size)
            with torch.no_grad():
                act = self.post(self.policy.select_action(self.pre(obs))).squeeze(0).cpu().numpy()
            acts.append(act)
            if i % 40 == 0:
                self.get_logger().info(f"  precalcul {i}/{len(self.obs)}  joints={np.round(act[:5],3)} grip={act[5]:.2f}")
        self.get_logger().info(f"PHASE 1 finie : {len(acts)} actions calculees.")
        if not self.a.go:
            self.get_logger().info("DRY : pas de rejeu (ajoute --go pour piloter le robot)."); return
        # ===== PHASE 2 : envoyer TOUTE la trajectoire d'UN COUP (spline lisse, comme roby_replay) =====
        # 1 point/15Hz en streaming => le JTC re-planifie a chaque point => saccades. UNE trajectoire
        # multi-points => le controleur execute une seule spline lissee. Lead-in doux vers le debut.
        lead = self.a.lead_in
        jt = JointTrajectory(); jt.joint_names = list(J)
        for k, act in enumerate(acts):
            pt = JointTrajectoryPoint(); pt.positions = [float(x) for x in act[:5]]
            tt = lead + k / self.a.hz
            pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
            jt.points.append(pt)
        # pince avec HYSTERESIS (evite le flottement autour de 0.5) : ferme >0.6, ouvre <0.4, sinon tient
        grips, g = [], False
        for act in acts:
            if act[5] > 0.6: g = True
            elif act[5] < 0.4: g = False
            grips.append(g)
        self.get_logger().info(f"PHASE 2 : envoi trajectoire unique ({len(acts)} pts, lead-in {lead:.0f}s "
                               f"+ {len(acts)/self.a.hz:.1f}s) vers le garde (le robot bouge)...")
        self.pub_traj.publish(jt)
        # joue la pince aux instants prevus (lead + k/hz) pendant l'execution
        t0 = time.monotonic(); last = None; k = 0
        while rclpy.ok() and k < len(grips):
            now = time.monotonic() - t0
            if now >= lead + k / self.a.hz:
                if grips[k] != last:
                    self.pub_grip.publish(Bool(data=grips[k])); last = grips[k]
                    self.get_logger().info(f"  [pince] {'FERME' if grips[k] else 'OUVRE'} @ {now:.1f}s")
                k += 1
            else:
                time.sleep(0.005)
        self.get_logger().info("fin du rejeu.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--bag", required=True, help="dossier ep_XXX du dataset")
    ap.add_argument("--go", action="store_true", help="PUBLIE vers le garde (sinon DRY)")
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--lead-in", type=float, default=3.0, help="transition douce vers le debut de l'episode (s)")
    a = ap.parse_args(); a.model = os.path.expanduser(a.model); a.bag = os.path.expanduser(a.bag)
    rclpy.init()
    node = InferBag(a)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
