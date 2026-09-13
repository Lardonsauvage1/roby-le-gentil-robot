#!/usr/bin/env python3
"""roby_eval_pince.py — juge le CANAL PINCE d'un modele, sans robot.

Pourquoi ce banc existe : le 2026-09-08, le bras fermait la pince EN MONTANT (donc trop
haut) pendant les rollouts autonomes. Le banc de teacher-forcing habituel
(`roby_eval_offline.py`) ne le voyait pas : il ne regarde que la PREMIERE action du chunk
et il annonçait "pince 100 %". Le defaut est ailleurs :

  1. le modele bascule la pince alors que le bras BOUGE, alors que le dataset est
     rigoureusement immobile (0,0 mm/s) a cet instant ;
  2. le canal pince CLIGNOTE : ~2x plus de bascules que la realite.

La pince est 1 dimension binaire parmi 7 continues, et elle ne change que 2 fois par
episode de ~300 frames : sa contribution a la perte L2 est negligeable. Le modele peut
donc etre excellent en position (2,5 mm) et mauvais sur la pince en meme temps. Il faut
un banc separe, sinon on ne le voit pas.

Deux mesures :
  --mode chunk   (defaut) inspecte les 8 actions predites autour de la fermeture et
                 mesure la vitesse du bras a l'instant de la bascule ; le dataset dit 0.
  --mode global  parcourt des episodes entiers et compte les desaccords et les bascules.

Usage :
  roby_eval_pince.py <pretrained_model> [--mode chunk|global] [--episodes 4]
                     [--dataset ~/lerobot-experiments/data_cache/<nom>]
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd
import torch

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))  # voisins de CE fichier, pas ceux du home
from roby_vision import image_keys, img_size_from_policy

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

DATASET = "~/lerobot-experiments/data_cache/lerobot_apple_propre_2cam_128"
SEUIL_MOUVEMENT = 8.0     # mm/s au-dela desquels on considere que le bras bouge


def charge(model):
    """-> (policy, pre, post, cles image, resolution)."""
    torch.set_num_threads(8)
    pol = DiffusionPolicy.from_pretrained(model).eval()
    pol.diffusion.num_inference_steps = 10
    pre, post = make_pre_post_processors(
        policy_cfg=pol.config, pretrained_path=model,
        preprocessor_overrides={"device_processor": {"device": "cpu"}})
    return pol, pre, post, image_keys(pol), img_size_from_policy(pol)


def image(cell, size):
    """PNG embarque dans le parquet -> tensor (1,3,size,size) float [0,1] RGB CHW.

    Meme pipeline que `roby_vision.decode_resize` (qui prend du JPEG ROS) : la
    resolution vient du checkpoint, jamais d'une constante.
    """
    a = cv2.imdecode(np.frombuffer(cell["bytes"], np.uint8), cv2.IMREAD_COLOR)
    a = cv2.resize(a, (size, size), interpolation=cv2.INTER_AREA)
    a = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)
    return (torch.from_numpy(a).float().permute(2, 0, 1) / 255.0).unsqueeze(0)


def observation(d, k, S, keys, R):
    obs = {"observation.state": torch.from_numpy(S[k].astype(np.float32)).unsqueeze(0)}
    for kk in keys:
        col = "observation.images.fixed" if "fixed" in kk else "observation.images.wrist"
        obs[kk] = image(d[col].iloc[k], R)
    return obs


def mode_chunk(df, pol, pre, post, keys, R, n_ep):
    """Le modele s'arrete-t-il avant de fermer, comme le fait le dataset ?"""
    vitesses = []
    for e in sorted(df.episode_index.unique())[:n_ep]:
        d = df[df.episode_index == e].sort_values("frame_index").reset_index(drop=True)
        A = np.stack(d["action"].values)
        S = np.stack(d["observation.state"].values)
        bascule = np.where(np.diff(A[:, 6]) > 0.5)[0]
        if not len(bascule):
            continue
        ferm = bascule[0] + 1
        print(f"--- episode {e} : consigne FERMER a la frame {ferm} ---")
        for k in range(ferm - 9, ferm + 4, 3):
            if k < 1:
                continue
            # Il faut DEUX observations dans la file (n_obs_steps=2) avant de juger le
            # chunk. Piege : chaque select_action consomme une action de la file. On
            # amorce donc sur k-1, on VIDE la file d'actions, puis on rejoue k : le
            # chunk est alors regenere en voyant bien (k-1, k). Sans le vidage on
            # analysait un chunk conditionne sur k-1 dupliquee, decale de 2 pas.
            pol.reset()
            with torch.no_grad():
                pol.select_action(pre(observation(d, k - 1, S, keys, R)))
                pol._queues["action"].clear()
                a0 = post(pol.select_action(pre(observation(d, k, S, keys, R))))
            chunk = np.stack([a0.reshape(-1).cpu().numpy()] +
                             [post(a if a.ndim == 2 else a.unsqueeze(0)).reshape(-1).cpu().numpy()
                              for a in list(pol._queues["action"])])
            g = chunk[:, 6]
            vit = np.r_[0, np.linalg.norm(np.diff(chunk[:, :3], axis=0), axis=1)] * 15 * 1000
            etat = "".join("F" if x > 0.5 else "." for x in g)
            quand = "avant" if k < ferm else "APRES"
            print(f"  f{k:3d} ({quand} la consigne) pince[{etat}]  vitesse du chunk "
                  f"{' '.join('%4.0f' % v for v in vit)} mm/s")
            flip = np.where(np.diff(g > 0.5))[0]
            if len(flip):
                i = flip[0] + 1
                v = vit[max(0, i - 1):i + 2].max()
                vitesses.append(v)
                verdict = "⚠️ FERME EN MOUVEMENT" if v > SEUIL_MOUVEMENT else "✅ immobile"
                print(f"        -> bascule a l'etape {i} du chunk, le bras bouge a "
                      f"{v:.0f} mm/s   {verdict}")
        print()
    if vitesses:
        v = np.array(vitesses)
        print(f"=== {len(v)} bascules : vitesse du bras {v.mean():.0f} mm/s en moyenne "
              f"(max {v.max():.0f}). Le dataset dit 0. ===")


def mode_global(df, pol, pre, post, keys, R, n_ep):
    """Combien de desaccords, et combien de bascules en trop, sur des episodes entiers ?"""
    tot = des = 0
    reelles = predites = 0
    for e in sorted(df.episode_index.unique())[:n_ep]:
        d = df[df.episode_index == e].sort_values("frame_index").reset_index(drop=True)
        A = np.stack(d["action"].values)
        S = np.stack(d["observation.state"].values)
        pol.reset()
        pred = []
        for k in range(len(d)):
            with torch.no_grad():
                a = post(pol.select_action(pre(observation(d, k, S, keys, R))))
            pred.append(a.reshape(-1).cpu().numpy()[6])
        p = np.array(pred) > 0.5
        v = A[:, 6] > 0.5
        bv = int((np.diff(v.astype(int)) != 0).sum())
        bp = int((np.diff(p.astype(int)) != 0).sum())
        tot += len(v); des += int((p != v).sum())
        reelles += bv; predites += bp
        print(f"ep{e:02d} : {bv} bascules reelles, {bp} predites, "
              f"desaccord {100 * (p != v).mean():5.1f} % des frames", flush=True)
    print(f"\n=== desaccord pince {100 * des / max(tot,1):.1f} % des frames ({des}/{tot})")
    print(f"    bascules : {reelles} reelles vs {predites} predites "
          f"(x{predites / max(reelles,1):.1f}) ===")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--mode", choices=["chunk", "global"], default="chunk")
    ap.add_argument("--episodes", type=int, default=4)
    ap.add_argument("--dataset", default=DATASET)
    a = ap.parse_args()
    a.model = os.path.expanduser(a.model)
    ds = os.path.expanduser(a.dataset)

    pol, pre, post, keys, R = charge(a.model)
    print(f"modele {os.path.basename(os.path.dirname(a.model))} | cameras={keys} | "
          f"R={R} | n_action_steps={pol.config.n_action_steps}\n")
    df = pd.read_parquet(os.path.join(ds, "data/chunk-000/file-000.parquet"))
    (mode_chunk if a.mode == "chunk" else mode_global)(
        df, pol, pre, post, keys, R, a.episodes)
    return 0


if __name__ == "__main__":
    sys.exit(main())
