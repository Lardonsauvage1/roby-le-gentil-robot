#!/usr/bin/env python3
"""roby_ov.py — inference du U-Net sur l'iGPU Intel Arc via OpenVINO.

Contexte (mesure 2026-07-22) : sur ce PC (Core Ultra 9 185H), le CPU deborde du
cache au-dela d'un certain U-Net -> le gros modele (263M) prend 2061 ms/inference,
impossible en temps reel. L'iGPU Arc a sa memoire dediee : le MEME modele tombe a
170 ms (x12). Fidelite validee : ecart 0.0064 sur l'action finale vs torch CPU.

On ne porte QUE le U-Net (96 % des params, 95 % du temps) sur l'iGPU. Le backbone
(resnet, leger), le scheduler et la normalisation restent dans le code LeRobot torch
CPU -> risque minimal, on ne touche pas a la logique validee.

Prerequis : openvino installe dans le venv (fait 2026-07-22). L'IR est genere une
fois par modele (fichier unet_ov.xml a cote du checkpoint) puis reutilise.
"""
import os
import numpy as np
import torch


def _patch_sinusoidal_f32():
    """L'embedding sinusoidal du timestep sort en float64 (torch.arange int64 *
    float python) ; OpenVINO refuse le melange f64/f32. On le force en f32. Ne change
    pas le resultat (juste la precision de l'embedding, deja negligeable)."""
    import lerobot.policies.diffusion.modeling_diffusion as mod
    orig = mod.DiffusionSinusoidalPosEmb.forward
    if getattr(orig, "_roby_f32", False):
        return
    def f32(self, x, _o=orig):
        return _o(self, x).float()
    f32._roby_f32 = True
    mod.DiffusionSinusoidalPosEmb.forward = f32


def _example_inputs(pol):
    """Entrees factices a la bonne forme pour tracer le U-Net."""
    cfg = pol.config
    H = cfg.horizon
    A = cfg.output_features["action"].shape[0]
    imk = [k for k in cfg.input_features if "image" in k][0]
    R = int(cfg.input_features[imk].shape[-1])
    NO = cfg.n_obs_steps
    SD = int(cfg.input_features["observation.state"].shape[-1])
    b = {imk: torch.rand(1, NO, 3, R, R),
         "observation.state": torch.rand(1, NO, SD),
         "observation.images": torch.rand(1, NO, 1, 3, R, R)}
    with torch.no_grad():
        gc = pol.diffusion._prepare_global_conditioning(b)
    return torch.rand(1, H, A), torch.tensor([1], dtype=torch.long), gc


def ensure_ir(pol, model_dir):
    """Genere l'IR OpenVINO du U-Net s'il n'existe pas. Retourne son chemin."""
    ir = os.path.join(model_dir, "unet_ov.xml")
    if os.path.exists(ir):
        return ir
    import openvino as ov
    _patch_sinusoidal_f32()
    unet = pol.diffusion.unet.eval()

    class W(torch.nn.Module):
        def __init__(s, u):
            super().__init__(); s.u = u
        def forward(s, x, ts, gc):
            return s.u(x, ts, global_cond=gc)

    ex = _example_inputs(pol)
    ovm = ov.convert_model(W(unet).eval(), example_input=ex)
    ov.save_model(ovm, ir)
    return ir


def patch_policy_igpu(pol, model_dir, device="GPU", logger=None):
    """Remplace le U-Net de `pol` par une version OpenVINO sur `device` (GPU=iGPU Arc,
    NPU, ou CPU). Fidelite validee. Retourne (ok, message)."""
    try:
        import openvino as ov
    except ImportError:
        return False, "openvino absent du venv"
    ir = ensure_ir(pol, model_dir)
    core = ov.Core()
    if device not in core.available_devices:
        return False, (f"device '{device}' indisponible (vus : {core.available_devices}). "
                       f"Pour le NPU, installer la lib userspace level-zero.")
    cm = core.compile_model(ir, device)
    out_port = cm.output(0)

    class OVUnet(torch.nn.Module):
        def forward(self, x, timestep, global_cond=None):
            r = cm({0: x.numpy(),
                    1: timestep.numpy().astype("int64"),
                    2: global_cond.numpy()})[out_port]
            return torch.from_numpy(r)

    pol.diffusion.unet = OVUnet()
    dev_name = core.get_property(device, "FULL_DEVICE_NAME")
    msg = f"U-Net porte sur {device} ({dev_name}) via OpenVINO"
    if logger:
        logger.warn(msg)
    return True, msg
