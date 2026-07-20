#!/usr/bin/env python3
"""roby_eval_offline.py — juge un modele SUR UN EPISODE ENREGISTRE, sans robot.

Le modele recoit exactement les observations qu'il aurait vues (image + etat lus dans
le bag) et on compare sa prediction a ce que l'oracle a REELLEMENT fait au pas suivant.
C'est du teacher-forcing : a chaque pas on repart de l'observation VRAIE, donc l'erreur
ne s'accumule pas -- on mesure la qualite de la politique, pas la derive en boucle
fermee.

Pourquoi c'est utile : jusqu'ici ce modele n'a jamais ete juge autrement qu'en le
laissant piloter le bras. Un echec live ne dit pas si c'est le MODELE ou la boucle
(latence, camera, pose de depart hors distribution). Ici on isole le modele, et ca ne
coute rien : ni robot, ni risque.

⚠️ Ce que ca ne dit PAS : la tenue en boucle fermee. Un modele bon en teacher-forcing
peut deriver une fois qu'il voit ses propres erreurs. C'est une condition necessaire,
pas suffisante.

Usage :
  roby_eval_offline.py --model <pretrained_model> --bag <ep_dir> [--hz 15] [--max 400]
"""
import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.expanduser("~"))
from roby_vision import decode_resize, image_keys, img_size_from_policy

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64MultiArray

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def lire_bag(bag):
    """-> listes (t, valeur) par flux. On garde les horodatages pour re-echantillonner."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    img, tcp, joints, grip = [], [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        ts = t / 1e9
        if topic == "/head_camera/left/image_raw/compressed":
            img.append((ts, bytes(deserialize_message(data, CompressedImage).data)))
        elif topic == "/tcp_pose":
            m = deserialize_message(data, Float64MultiArray)
            if len(m.data) >= 6:
                tcp.append((ts, np.array(m.data[:6], float)))
        elif topic == "/joint_states":
            m = deserialize_message(data, JointState)
            d = dict(zip(m.name, m.position))
            if all(j in d for j in J):
                joints.append((ts, np.array([float(d[j]) for j in J], float)))
        elif topic == "/gripper":
            grip.append((ts, bool(deserialize_message(data, Bool).data)))
    return img, tcp, joints, grip


def au_plus_proche(seq, t):
    """valeur de seq la plus proche de t (seq triee par temps)."""
    if not seq:
        return None
    i = min(range(len(seq)), key=lambda k: abs(seq[k][0] - t))
    return seq[i][1]


def etat_pince(grip, t):
    """dernier etat de pince connu a l'instant t (evenementiel : 3 messages/episode)."""
    v = False
    for (ts, g) in grip:
        if ts <= t:
            v = g
        else:
            break
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--hz", type=float, default=15.0, help="cadence du dataset")
    ap.add_argument("--max", type=int, default=400, help="nb max de pas evalues")
    a = ap.parse_args()
    a.model = os.path.expanduser(a.model)
    a.bag = os.path.expanduser(a.bag)

    torch.set_num_threads(6)
    pol = DiffusionPolicy.from_pretrained(a.model).eval()
    cfg = pol.config
    pol.diffusion.num_inference_steps = 10
    pre, post = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=a.model,
        preprocessor_overrides={"device_processor": {"device": "cpu"}})
    dev = torch.device("cpu")
    keys = image_keys(pol)
    R = img_size_from_policy(pol)
    sdim = int(cfg.input_features["observation.state"].shape[-1])
    adim = int(cfg.output_features["action"].shape[-1])
    cart = (sdim, adim) == (6, 7)
    nparam = sum(p.numel() for p in pol.parameters())

    print(f"modele  : {os.path.basename(os.path.dirname(a.model))}  "
          f"({nparam/1e6:.0f}M, R={R}, etat={sdim}, action={adim}, "
          f"{'CARTESIEN' if cart else 'JOINT'})")
    print(f"episode : {os.path.basename(a.bag)}")

    img, tcp, joints, grip = lire_bag(a.bag)
    print(f"          {len(img)} images, {len(tcp)} tcp, {len(joints)} joints, "
          f"{len(grip)} evts pince")
    if not img:
        print("❌ aucune image dans ce bag"); return 1
    etats = tcp if cart else joints
    if not etats:
        print(f"❌ ce bag n'a pas l'etat requis "
              f"({'/tcp_pose' if cart else '/joint_states'})"); return 1

    # re-echantillonnage a la cadence du dataset, sur la base des images
    t0, t1 = img[0][0], img[-1][0]
    pas = 1.0 / a.hz
    temps = [t0 + k * pas for k in range(int((t1 - t0) / pas))][:a.max]
    print(f"          {len(temps)} pas evalues a {a.hz:.0f} Hz\n")

    pol.reset()
    err_pos, err_grip, n_grip = [], 0, 0
    pred_z, vrai_z = [], []
    for k, t in enumerate(temps[:-1]):
        jpeg = au_plus_proche(img, t)
        etat = au_plus_proche(etats, t)
        vrai_suivant = au_plus_proche(etats, temps[k + 1])
        if jpeg is None or etat is None or vrai_suivant is None:
            continue
        obs = {keys[0]: decode_resize(jpeg, dev, R),
               "observation.state": torch.from_numpy(etat.astype(np.float32)).unsqueeze(0)}
        with torch.no_grad():
            act = post(pol.select_action(pre(obs))).squeeze(0).cpu().numpy()

        if cart:
            err_pos.append(np.linalg.norm(act[:3] - vrai_suivant[:3]))
            pred_z.append(act[2]); vrai_z.append(vrai_suivant[2])
            g_pred = bool(act[6] > 0.5)
        else:
            err_pos.append(np.linalg.norm(act[:5] - vrai_suivant))
            g_pred = bool(act[5] > 0.5)
        g_vrai = etat_pince(grip, temps[k + 1])
        n_grip += 1
        err_grip += int(g_pred != g_vrai)

    if not err_pos:
        print("❌ aucun pas exploitable"); return 1
    e = np.array(err_pos)
    unite = "mm" if cart else "mrad"
    fac = 1000.0
    print("=== ECART entre ce que le modele PREDIT et ce que l'oracle a FAIT ===")
    print(f"  mediane : {np.median(e)*fac:7.2f} {unite}")
    print(f"  moyenne : {e.mean()*fac:7.2f} {unite}")
    print(f"  90e pct : {np.percentile(e,90)*fac:7.2f} {unite}")
    print(f"  max     : {e.max()*fac:7.2f} {unite}")
    print(f"\n=== PINCE ===")
    print(f"  desaccords : {err_grip}/{n_grip}  ({100.0*err_grip/max(n_grip,1):.1f} %)")
    if cart and pred_z:
        pz, vz = np.array(pred_z), np.array(vrai_z)
        print(f"\n=== DESCENTE (z) ===")
        print(f"  z reel   : {vz.max():.3f} -> {vz.min():.3f} m")
        print(f"  z predit : {pz.max():.3f} -> {pz.min():.3f} m")
        print(f"  correlation predit/reel : {np.corrcoef(pz, vz)[0,1]:+.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
