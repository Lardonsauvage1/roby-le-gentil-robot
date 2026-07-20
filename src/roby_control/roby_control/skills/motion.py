"""
Couche MOUVEMENT : bascule SIM / RÉEL pour les skills de manipulation.

⚠️ DÉFAUT = SIM : rien ne bouge, on imprime seulement. On passe en RÉEL
explicitement (flag --real du runner) → et JAMAIS sans validation de Sam.

RÉEL réutilise l'existant (pas de réécriture) :
  - déplacements libres MoveIt (anti-collision) : sous-processus roby_moveit_seq.py
    qui va, dans l'ordre, aux poses nommées passées en argument.
  - pince / verrou tête : publication ROS2 sur /gripper et /head_lock.

Les fonctions renvoient un `Job` (asynchrone) : l'action py_trees le démarre dans
initialise(), puis interroge job.poll() à chaque tick (None = en cours, sinon code
retour) — c'est ce qui donne le RUNNING sans geler l'arbre.
"""
import os
import subprocess

SIM = True          # bascule globale (le runner --real met False)
SIM_TICKS = 3       # "durée" simulée d'un mouvement, en ticks

HOME = os.path.expanduser("~")
# Python SYSTÈME de ROS Jazzy (3.12). Surtout PAS sys.executable (venv du projet)
# ni `python3` (pyenv du profil) → rclpy est compilé pour le 3.12 système.
ROS_PY = "/usr/bin/python3"


class Job:
    """Travail asynchrone : soit un vrai sous-processus, soit un compte à rebours SIM."""
    def __init__(self, popen=None, sim_ticks=SIM_TICKS, label=""):
        self.popen = popen
        self.sim_left = sim_ticks
        self.label = label

    def poll(self):
        """None = encore en cours ; int = terminé (0 = OK)."""
        if self.popen is not None:
            return self.popen.poll()
        self.sim_left -= 1
        return None if self.sim_left > 0 else 0


MOTION_LOG = "/tmp/roby_motion.log"


def _sh_job(cmd, label):
    """Lance une commande shell détachée et l'emballe en Job (RÉEL). Logue la sortie."""
    logf = open(MOTION_LOG, "a")
    logf.write(f"\n===== {label} =====\n{cmd}\n")
    logf.flush()
    # bash -c (PAS -lc) : on n'ouvre pas de shell de login → pas de réactivation
    # pyenv (qui remplacerait le python 3.12 de ROS par 3.11 → rclpy cassé).
    # L'env hérité (ROS déjà sourcé par le runner) fournit ros2 / PYTHONPATH.
    p = subprocess.Popen(["bash", "-c", cmd], env=dict(os.environ),
                         stdout=logf, stderr=subprocess.STDOUT)
    return Job(popen=p, label=label)


def free_sequence(pose_names, vel=None):
    """Déplacements libres MoveIt en passant par les poses nommées, dans l'ordre."""
    label = " -> ".join(pose_names)
    if SIM:
        print(f"   ▶ [SIM] libre MoveIt : {label}")
        return Job(label=label)
    names = " ".join(pose_names)
    velopt = f"--vel {vel}" if vel else ""
    cmd = f"{ROS_PY} {HOME}/roby_moveit_seq.py {velopt} {names}"
    print(f"   ▶ [RÉEL] libre MoveIt : {label}")
    return _sh_job(cmd, label)


import re

JOINT_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def read_joint_positions(timeout=8):
    """Position CRUE du contrôleur via /joint_states (liste de 5 floats), None si échec."""
    try:
        p = subprocess.run(
            ["bash", "-c",
             f"ros2 topic echo /joint_states --once --field position"],
            env=dict(os.environ), capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    m = re.search(r"\[([^\]]+)\]", p.stdout)
    if not m:
        return None
    try:
        vals = [float(x) for x in m.group(1).split(",")]
        return vals if len(vals) == 5 else None
    except ValueError:
        return None


def alignment_error(expected_joints):
    """Écart max (rad) entre la croyance du contrôleur et une pose attendue.
    Renvoie (err, believed) ; err = None si /joint_states illisible."""
    believed = read_joint_positions()
    if believed is None:
        return None, None
    err = max(abs(believed[i] - expected_joints[i]) for i in range(5))
    return err, believed


def set_gripper(closed):
    """Pince : True=fermer(55°), False=ouvrir(120°) — via /gripper (std_msgs/Bool)."""
    lab = "fermer" if closed else "ouvrir"
    if SIM:
        print(f"   ▶ [SIM] pince : {lab}")
        return Job(label=lab, sim_ticks=1)
    val = "true" if closed else "false"
    cmd = f'ros2 topic pub --once /gripper std_msgs/msg/Bool "{{data: {val}}}"'
    print(f"   ▶ [RÉEL] pince : {lab}")
    return _sh_job(cmd, lab)


def set_head_lock(lock):
    """Verrou tête via /head_lock (std_msgs/Bool).
    Autorité = roby_system.cpp (on_head_lock, calibré 2026-07-07) :
      /head_lock true  -> 50° = VERROUILLÉ
      /head_lock false -> 75° = DÉVERROUILLÉ
    (⚠️ le standalone head_lock_node.py a ses labels inversés — NE PAS s'y fier.)"""
    lab = "verrouiller" if lock else "déverrouiller"
    if SIM:
        print(f"   ▶ [SIM] verrou tête : {lab}")
        return Job(label=lab, sim_ticks=1)
    val = "true" if lock else "false"     # true=verrouiller (50°), false=déverrouiller (75°)
    cmd = f'ros2 topic pub --once /head_lock std_msgs/msg/Bool "{{data: {val}}}"'
    print(f"   ▶ [RÉEL] verrou tête : {lab} (/head_lock {val})")
    return _sh_job(cmd, lab)
