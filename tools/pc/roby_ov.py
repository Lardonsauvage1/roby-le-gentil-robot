#!/usr/bin/env python3
"""roby_ov.py — inference du U-Net sur l'iGPU Intel Arc via OpenVINO.

Contexte, RE-MESURE le 2026-09-09 a protocole propre (memes observations,
5 iterations de chauffe JETEES puis 30 mesurees, machine au repos), modele 263M :

    torch CPU              393,3 ms  (ecart-type 34,7 | mediane 376 | max 500)
    iGPU Arc / OpenVINO    170,9 ms  (ecart-type 10,9 | mediane 166 | max 200)
    -> l'iGPU est x2,3 plus rapide, et surtout 3x plus REGULIER

⚠️ La note precedente annoncait "CPU 2061 ms, donc x12". Ce chiffre etait FAUX : il
venait d'une mesure unique sans chauffe. Le 170 ms de l'iGPU, lui, s'est confirme au
milli-seconde pres. Le vrai facteur est x2,3, pas x12.

Ce qui compte en pratique n'est pas tant la moyenne que le MAX : le budget temps reel
est de 533 ms (n_action_steps 8 a 15 Hz). Le CPU y monte a 500 ms au banc -- et
428-588 ms en conditions reelles avec ROS et les cameras, donc il DEPASSE. L'iGPU
plafonne a 200 ms. C'est la marge, pas la vitesse, qui justifie ce portage.

Une partie du gain vient de la precision : l'IR OpenVINO est en f16/f32 mixte (bin de
855 Mo pour un U-Net qui pese ~1 Go en f32), donc moins d'octets a lire.
Fidelite validee : ecart 0.0064 sur l'action finale vs torch CPU.

Contrepartie a garder en tete : l'iGPU occupe par l'inference n'est plus disponible
pour entrainer en parallele (boucle DAgger).

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
    # ⚠️ TOUTES les cles image, pas seulement la premiere (2026-09-07). Avec un modele
    # bi-camera, ne tracer qu'une vue produisait un global_cond de la mauvaise taille et
    # la conversion echouait sur "mat1 and mat2 shapes cannot be multiplied (1x268 and 396x1024)".
    imks = [k for k in cfg.input_features if "image" in k]
    R = int(cfg.input_features[imks[0]].shape[-1])
    NO = cfg.n_obs_steps
    SD = int(cfg.input_features["observation.state"].shape[-1])
    b = {k: torch.rand(1, NO, 3, R, R) for k in imks}
    b["observation.state"] = torch.rand(1, NO, SD)
    b["observation.images"] = torch.rand(1, NO, len(imks), 3, R, R)
    with torch.no_grad():
        gc = pol.diffusion._prepare_global_conditioning(b)
    return torch.rand(1, H, A), torch.tensor([1], dtype=torch.long), gc


def _poids(model_dir):
    """Fichier de poids du modele (a cote de l'export, ou dans pretrained_model/)."""
    for d in (model_dir, os.path.join(model_dir, "pretrained_model")):
        f = os.path.join(d, "model.safetensors")
        if os.path.exists(f):
            return f
    return None


def ensure_ir(pol, model_dir):
    """Genere l'IR OpenVINO du U-Net s'il n'existe pas ou s'il est PLUS ANCIEN que les
    poids. Retourne son chemin.

    L'export etait reutilise des qu'il existait : un checkpoint re-sauve au meme endroit
    (reprise, DAgger) faisait tourner sur l'iGPU l'ANCIEN U-Net avec le nouvel encodeur
    et la nouvelle normalisation, sans rien dire (revue du 2026-09-13). L'ecriture passe
    maintenant par un fichier temporaire renomme (deux exports simultanes ne corrompent
    plus le fichier)."""
    ir = os.path.join(model_dir, "unet_ov.xml")
    if os.path.exists(ir):
        w = _poids(model_dir)
        if w is None or os.path.getmtime(ir) >= os.path.getmtime(w):
            return ir
        print("[roby_ov] export OpenVINO PLUS ANCIEN que les poids : re-export", flush=True)
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
    tmp = os.path.join(model_dir, ".unet_ov.tmp%d.xml" % os.getpid())
    ov.save_model(ovm, tmp)
    os.replace(tmp[:-4] + ".bin", ir[:-4] + ".bin")    # .bin d'abord : le .xml fait foi
    os.replace(tmp, ir)
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
