"""
État cru du monde (belief state) pour les Behavior Trees de Roby.

⚠️ OPEN-LOOP : aucun capteur ne mesure "une tête est montée" ou "un objet est
pincé". Cet état est donc *cru*, pas *mesuré* : il est mis à jour par les EFFETS
des actions, pas par des capteurs. Si tu bouges le robot à la main ou coupes la
stack, l'état diverge du réel → utilise la commande `reset` du runner pour le
resynchroniser sur une pose connue (le nid).

On stocke l'état dans le BLACKBOARD de py_trees (un tableau clé/valeur partagé
par tous les nœuds de l'arbre) et on le persiste sur disque en YAML entre deux
appels.
"""
import os
import yaml
import py_trees

# Fichier de persistance (l'état survit entre deux lancements du runner)
WORLD_PATH = os.path.expanduser("~/roby_world.yaml")

# Les clés de l'état + leurs valeurs par défaut (état "propre" : au nid, sans tête)
DEFAULT_STATE = {
    "head": None,          # None | "pince" | "ventouse"   (tête montée)
    "gripper": "open",     # "open" | "closed"
    "holding": False,      # True si un objet est agrippé
    "rack": {              # ce qui reste rangé au rack : "present" | "empty"
        "pince": "present",
        "ventouse": "present",
    },
    "at": "nid",           # zone où se croit le bras : "nid" | "approche_nid" | "libre"
}

# Ordre stable pour l'affichage
KEYS = ["head", "gripper", "holding", "rack", "at"]


def _client():
    """Un client blackboard qui a le droit de lire/écrire toutes les clés d'état."""
    bb = py_trees.blackboard.Client(name="World")
    for k in KEYS:
        bb.register_key(key=k, access=py_trees.common.Access.WRITE)
    return bb


def load_world(path=WORLD_PATH):
    """Charge l'état depuis le disque, ou l'état par défaut si absent."""
    if os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        # complète les clés manquantes avec les défauts
        state = dict(DEFAULT_STATE)
        state.update({k: data.get(k, DEFAULT_STATE[k]) for k in KEYS})
        return state
    return dict(DEFAULT_STATE)


def seed_blackboard(state):
    """Copie l'état (dict) dans le blackboard partagé, avant de ticker l'arbre."""
    bb = _client()
    for k in KEYS:
        setattr(bb, k, state[k])
    return bb


def dump_blackboard():
    """Relit l'état depuis le blackboard (après tick) sous forme de dict."""
    bb = _client()
    return {k: getattr(bb, k) for k in KEYS}


def save_world(state, path=WORLD_PATH):
    """Persiste l'état sur disque."""
    with open(path, "w") as f:
        yaml.safe_dump(state, f, allow_unicode=True, sort_keys=False)


def pretty(state):
    """Ligne lisible de l'état, pour les logs."""
    return (f"head={state['head']} | gripper={state['gripper']} | "
            f"holding={state['holding']} | at={state['at']} | rack={state['rack']}")
