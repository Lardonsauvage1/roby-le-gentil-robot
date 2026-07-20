"""
Carte sémantique des positions : relie les NOMS logiques (nid, approches,
logements de têtes) aux poses articulaires enregistrées dans ~/roby_poses.yaml.

⚠️ Source unique des valeurs : ~/roby_poses.yaml (partagé avec roby_moveit_seq.py
et roby_tool_pickup.py). Ici on ne stocke QUE des noms de poses — jamais les
radians en dur — pour ne pas dupliquer/désynchroniser.

Architecture (réponses Sam 2026-07-07) :
  - NID = dock de référence unique (origine), avec son point d'approche.
  - RACK séparé : chaque tête a SON logement + SON point d'approche.
  - 2 têtes physiques : pince, ventouse.
"""
import os
import yaml

POSES_FILE = os.path.expanduser("~/roby_poses.yaml")

# Le nid : dock de référence + son approche
NEST = {"dock": "nid", "approach": "approche_nid"}

# Les têtes : type -> {logement au rack, point d'approche du logement}
# pince = le "changeur_outil" déjà enregistré. ventouse = À CAPTURER.
HEADS = {
    "pince":    {"rack": "changeur_outil", "approach": "approche_changeur"},
    "ventouse": {"rack": "rack_ventouse",  "approach": "approche_ventouse"},
}


def load_poses(path=POSES_FILE):
    """Charge {nom: [j1..j5]} depuis roby_poses.yaml."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {k: [float(x) for x in v] for k, v in data.items()
            if isinstance(v, (list, tuple))}


def pose_exists(name):
    return name in load_poses()


def missing_poses():
    """Liste des poses référencées par la carte mais absentes du YAML."""
    have = load_poses()
    needed = [NEST["dock"], NEST["approach"]]
    for h in HEADS.values():
        needed += [h["rack"], h["approach"]]
    return [n for n in needed if n not in have]
