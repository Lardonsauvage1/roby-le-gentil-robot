"""
Capteurs réels de la stack : ce module LIT la réalité (ROS graph, ping, horloge).

Contrairement aux skills de manipulation (état *cru*, open-loop), ici tout est
MESURÉ. Un `Snapshot` est pris une fois par tick et rangé dans le blackboard ;
les checks (checks.py) lisent ce snapshot au lieu de relancer 15 commandes.
"""
import os
import subprocess
import py_trees
from . import stack_spec as S


def sh(cmd, timeout=6):
    """Lance une commande shell, renvoie (rc, stdout). Ne lève jamais."""
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout, executable="/bin/bash")
        return p.returncode, p.stdout.strip()
    except subprocess.TimeoutExpired:
        return 124, ""
    except Exception as e:
        return 1, str(e)


def _ros_env():
    """Env avec ROS sourcé + DDS, pour les commandes ros2 CLI."""
    # On suppose que l'utilisateur a déjà sourcé ROS avant de lancer le runner ;
    # on renforce juste les variables DDS critiques.
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "42")
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    return env


def ros_nodes(timeout=8):
    """Ensemble des nodes visibles sur le graphe ROS."""
    rc, out = sh("ros2 node list", timeout=timeout)
    return set(l.strip() for l in out.splitlines() if l.strip()) if rc == 0 else set()


def topic_counts(topic, timeout=6):
    """(#publishers, #subscribers) d'un topic, via `ros2 topic info`."""
    rc, out = sh(f"ros2 topic info {topic}", timeout=timeout)
    pub = sub = 0
    for line in out.splitlines():
        if "Publisher count:" in line:
            pub = int(line.split(":")[1])
        elif "Subscription count:" in line:
            sub = int(line.split(":")[1])
    return pub, sub


def topic_flowing(topic, timeout=7):
    """
    Vrai si AU MOINS un message arrive sur le topic dans le délai imparti.
    Plus robuste que `ros2 topic hz` pour un check de santé (pas de fenêtre à
    remplir ; on laisse le temps de la découverte DDS). `echo --once` sort dès
    le 1er message reçu.
    """
    rc, out = sh(f"timeout {timeout-1} ros2 topic echo {topic} --once --no-daemon",
                 timeout=timeout)
    # rc 0 = un message reçu et affiché ; sortie non vide = ça circule
    return rc == 0 and bool(out.strip())


def ping(host, timeout=3):
    if host == "local":
        return True
    rc, _ = sh(f"ping -c1 -W2 {host}", timeout=timeout)
    return rc == 0


def clock_skew(pi5_ip, timeout=5):
    """Désynchro horloge PC<->Pi5 en secondes (None si Pi5 injoignable)."""
    rc, out = sh(f"ssh -o ConnectTimeout=4 roby@{pi5_ip} 'date +%s'", timeout=timeout)
    if rc != 0 or not out.isdigit():
        return None
    rc2, here = sh("date +%s", timeout=3)
    return abs(int(here) - int(out))


def dds_iface_ok(config_path, iface):
    """Vrai si le cyclone_config.xml pointe bien la bonne interface réseau."""
    if not os.path.exists(config_path):
        return False
    with open(config_path) as f:
        return f'name="{iface}"' in f.read()


class Snapshot(py_trees.behaviour.Behaviour):
    """
    Feuille de DÉTECTION : prend une photo de la réalité et la range au
    blackboard (clé 'snap'). Toujours SUCCESS — elle observe, ne juge pas.
    Placée en tête de l'arbre, elle alimente tous les checks du tick.
    """
    def __init__(self):
        super().__init__(name="📷 snapshot stack")
        self.bb = self.attach_blackboard_client(name="snapshot")
        self.bb.register_key(key="snap", access=py_trees.common.Access.WRITE)

    def update(self):
        nodes = ros_nodes()

        # Paires (topic, mode) dont les specs ont besoin — un MÊME topic peut être
        # surveillé de 2 façons (ex. /joint_states : 'pub' par jsb ET 'flow' par
        # ros2_control), donc on déduplique par la PAIRE, pas par le topic seul.
        needed = set()
        for svc in S.STACK:
            live = svc.get("live")
            if live:
                needed.add((live["topic"], live["kind"]))

        counts, flow = {}, {}
        for topic, kind in needed:
            if kind == "flow":
                flow[topic] = topic_flowing(topic)
            else:  # pub / sub
                counts[topic] = topic_counts(topic)

        snap = {
            "nodes": nodes,
            "counts": counts,          # topic -> (pub, sub)
            "flow": flow,              # topic -> bool (ça circule ?)
            "ping_pi5": ping(S.INFRA["pi5_ip"]),
            "clock_skew": clock_skew(S.INFRA["pi5_ip"]),
            "dds_ok": dds_iface_ok(S.INFRA["pc_dds_config"], S.INFRA["pc_dds_iface"]),
        }
        self.bb.snap = snap
        return py_trees.common.Status.SUCCESS
