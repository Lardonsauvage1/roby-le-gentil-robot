#!/usr/bin/env python3
"""roby_vision.py — prétraitement image PARTAGÉ pour l'inférence.

Raison d'être : ce prétraitement était recopié à l'identique dans 5 fichiers
(`roby_infer.py`, `roby_infer_cart.py`, `roby_infer_bag.py`, et 2 nœuds désormais
archivés). Les copies n'avaient PAS divergé sur le calcul des pixels — mais trois
d'entre elles avaient figé la résolution à `IMG = 224` en dur, ce qui les rendait
fausses (ou plantantes) sur les modèles actuels en 96/128.

C'est le mode d'échec qui coûte le plus cher : si l'image d'entrée diffère de celle
vue à l'entraînement, le réseau se dégrade **sans lever la moindre erreur**. On
cherche alors le problème du côté du robot ou du modèle pendant des heures.

Règle : la résolution ne doit JAMAIS être une constante — elle se lit dans le
checkpoint, via `img_size_from_policy()`.

Le pipeline doit rester identique à celui du dataset :
    JPEG -> imdecode (BGR) -> resize R×R INTER_AREA -> BGR2RGB -> CHW -> /255
Pas de rotation (les images arrivent déjà orientées par `cam_pub_pi2_dual.py`).
Pas de center-crop manuel : les modèles qui ont un `crop_shape` l'appliquent
eux-mêmes en `eval()`.
"""
import cv2
import numpy as np
import torch


def decode_resize(jpeg, dev, size):
    """JPEG compressé -> tensor (1, 3, size, size) float [0,1] RGB CHW.

    `size` est la résolution DU MODÈLE : la passer explicitement (jamais de constante
    globale) est ce qui empêche de ré-introduire un 224 en dur.
    """
    arr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)   # BGR 480x640
    arr = cv2.resize(arr, (size, size), interpolation=cv2.INTER_AREA)
    arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
    return (torch.from_numpy(arr).float().permute(2, 0, 1) / 255.0).unsqueeze(0).to(dev)


def image_keys(policy):
    """Clés d'observation image attendues par le checkpoint.

    1 clé (`observation.images.fixed`) pour les modèles b2* mono-caméra ;
    2 clés pour l'ancienne génération. Toujours tester la longueur avant d'indexer
    `[1]` : un accès inconditionnel plantait sur les modèles mono-caméra.
    """
    return [k for k in policy.config.input_features if "image" in k]


def img_size_from_policy(policy, override=0):
    """Résolution attendue, lue dans le checkpoint. `override > 0` force une valeur
    (utile pour un test ponctuel ; à éviter en production)."""
    if override and override > 0:
        return int(override)
    keys = image_keys(policy)
    if not keys:
        raise ValueError("ce checkpoint n'attend aucune image")
    return int(policy.config.input_features[keys[0]].shape[-1])   # (C, R, R)
