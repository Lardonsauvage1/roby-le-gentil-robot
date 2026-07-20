"""
Feuilles d'action MOTORISÉES (asynchrones) + petites feuilles d'état.

Motorisées : démarrent un Job (motion.py) dans initialise(), renvoient RUNNING
tant que job.poll() est None, puis appliquent les EFFETS (état cru) et SUCCESS.
En SIM elles n'impriment que ; en RÉEL elles pilotent le vrai bras / les servos.

État (instantanées) : mettent juste à jour le belief après qu'un mouvement
physique a eu lieu (ex. head verrouillée -> head=<type>, logement vidé).
"""
import py_trees
from py_trees.common import Status
from . import motion


# ---------------- actions motorisées (async) ----------------
class _MotionAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, rw_keys):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=name)
        for k in rw_keys:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)
        self._job = None

    def start(self):
        raise NotImplementedError      # -> renvoie un motion.Job

    def effects(self):
        pass                            # -> maj belief après succès

    def initialise(self):
        self._job = self.start()

    def update(self):
        rc = self._job.poll()
        if rc is None:
            return Status.RUNNING
        if rc != 0:
            print(f"   ❌ {self.name} : échec (code {rc})")
            return Status.FAILURE
        self.effects()
        return Status.SUCCESS


class MoveVia(_MotionAction):
    """Déplacement libre MoveIt via une suite de poses nommées ; arrive à `at`."""
    def __init__(self, pose_names, arrive_at):
        super().__init__(name="aller " + " → ".join(pose_names), rw_keys=["at"])
        self.pose_names = list(pose_names)
        self.arrive_at = arrive_at

    def start(self):
        return motion.free_sequence(self.pose_names)

    def effects(self):
        self.bb.at = self.arrive_at


class Grip(_MotionAction):
    """Pince : closed=True → fermer, False → ouvrir. Met à jour l'état gripper."""
    def __init__(self, closed):
        super().__init__(name="pince " + ("fermer" if closed else "ouvrir"),
                         rw_keys=["gripper"])
        self.closed = closed

    def start(self):
        return motion.set_gripper(self.closed)

    def effects(self):
        self.bb.gripper = "closed" if self.closed else "open"


class HeadLock(_MotionAction):
    """Verrou de tête : lock=True → verrouiller (fixe la tête sur la monture)."""
    def __init__(self, lock):
        super().__init__(name="verrou " + ("ON" if lock else "OFF"), rw_keys=[])
        self.lock = lock

    def start(self):
        return motion.set_head_lock(self.lock)


# ---------------- feuilles d'état (instantanées) ----------------
class SetHead(py_trees.behaviour.Behaviour):
    """Belief : tête `which` désormais montée, son logement au rack devient vide."""
    def __init__(self, which):
        super().__init__(name=f"état: tête={which}")
        self.which = which
        self.bb = self.attach_blackboard_client(name="sethead")
        self.bb.register_key(key="head", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="rack", access=py_trees.common.Access.WRITE)

    def update(self):
        self.bb.head = self.which
        rack = dict(self.bb.rack)
        rack[self.which] = "empty"
        self.bb.rack = rack
        return Status.SUCCESS


class ClearHead(py_trees.behaviour.Behaviour):
    """Belief : tête montée reposée → son logement redevient plein, plus de tête."""
    def __init__(self):
        super().__init__(name="état: tête=None")
        self.bb = self.attach_blackboard_client(name="clearhead")
        self.bb.register_key(key="head", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="rack", access=py_trees.common.Access.WRITE)

    def update(self):
        which = self.bb.head
        if which is not None:
            rack = dict(self.bb.rack)
            rack[which] = "present"
            self.bb.rack = rack
        self.bb.head = None
        return Status.SUCCESS


class SetHolding(py_trees.behaviour.Behaviour):
    """Belief : on tient (True) ou on relâche (False) un objet."""
    def __init__(self, val):
        super().__init__(name=f"état: holding={val}")
        self.val = val
        self.bb = self.attach_blackboard_client(name="setholding")
        self.bb.register_key(key="holding", access=py_trees.common.Access.WRITE)

    def update(self):
        self.bb.holding = self.val
        return Status.SUCCESS
