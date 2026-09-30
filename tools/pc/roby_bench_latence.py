#!/usr/bin/env python3
"""Mesure la latence d'inference CPU des modeles du balayage.

C'est LA metrique qui compte pour le temps-reel : le deploiement tourne sur CPU
(OpenVINO), pas sur l'iGPU. La vitesse d'ENTRAINEMENT sur XPU ne la predit pas.

Budget temps-reel du robot : re-inference tous les 8 pas a 15 Hz = ~0,53 s.
Reference INDEX.md : num_inference_steps = 10 obligatoire.

Lancer : ~/roby_bench_latence.sh --out ~/lerobot-experiments/outputs/cart_cooldown_20260728
"""
import argparse
import json
import statistics
import time
from pathlib import Path

import torch


def bench(model_dir, n_warmup, n_iter, steps_diff):
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

    pol = DiffusionPolicy.from_pretrained(model_dir).eval().to("cpu")
    pol.diffusion.num_inference_steps = steps_diff
    cfg = pol.config

    # observation factice au bon format
    obs = {}
    for k, f in cfg.input_features.items():
        shape = tuple(f.shape)
        if "image" in k:
            obs[k] = torch.rand(1, *shape)
        else:
            obs[k] = torch.zeros(1, *shape)

    lat = []
    with torch.no_grad():
        for i in range(n_warmup + n_iter):
            pol.reset()
            t0 = time.perf_counter()
            pol.select_action({k: v.clone() for k, v in obs.items()})
            dt = time.perf_counter() - t0
            if i >= n_warmup:
                lat.append(dt)
    n_par = sum(p.numel() for p in pol.parameters()) / 1e6
    return n_par, lat


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True, help="dossier contenant <taille>/pretrained_model*")
    p.add_argument("--variante", default="pretrained_model",
                   choices=["pretrained_model", "pretrained_model_ema", "pretrained_model_avg"])
    p.add_argument("--iter", type=int, default=12)
    p.add_argument("--warmup", type=int, default=3)
    p.add_argument("--steps-diff", type=int, default=10)
    p.add_argument("--budget", type=float, default=0.53, help="budget temps-reel en s")
    a = p.parse_args()

    root = Path(a.out).expanduser()
    res = []
    for d in sorted(root.iterdir()):
        md = d / a.variante
        if not (md / "model.safetensors").is_file():
            continue
        print(f"  mesure {d.name} ...", flush=True)
        try:
            n_par, lat = bench(md, a.warmup, a.iter, a.steps_diff)
        except Exception as e:
            print(f"    ECHEC {d.name}: {type(e).__name__}: {e}")
            continue
        res.append({"nom": d.name, "params_M": round(n_par, 1),
                    "median_s": round(statistics.median(lat), 3),
                    "min_s": round(min(lat), 3), "max_s": round(max(lat), 3)})

    res.sort(key=lambda r: r["median_s"])
    print(f"\nLatence CPU, {a.steps_diff} pas de diffusion, variante '{a.variante}'")
    print(f"budget temps-reel = {a.budget:.2f} s\n")
    print(f"{'modele':8s} {'params':>9s} {'mediane':>9s} {'min':>8s} {'max':>8s}   temps-reel ?")
    for r in res:
        ok = "OUI" if r["median_s"] <= a.budget else "NON"
        print(f"{r['nom']:8s} {r['params_M']:8.1f}M {r['median_s']:8.3f}s {r['min_s']:7.3f}s "
              f"{r['max_s']:7.3f}s   {ok}")
    (root / f"latence_{a.variante}.json").write_text(json.dumps(res, indent=2))
    print(f"\nresultats : {root}/latence_{a.variante}.json")


if __name__ == "__main__":
    main()
