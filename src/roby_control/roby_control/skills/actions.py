"""
Feuilles "ACTION" : elles AGISSENT et mettent à jour l'état (effets).

⚠️ MODE SIMULATION : pour l'instant elles ne bougent AUCUN moteur. Elles se
contentent d'imprimer ce qu'elles feraient et de simuler une durée. C'est
volontaire : on valide d'abord la logique de l'arbre à sec.

Le motif important = ASYNCHRONE via RUNNING :
  - au 1er tick : on "démarre" le mouvement (ici juste un compteur),
  - aux ticks suivants : on renvoie RUNNING tant que ce n'est pas fini
    (l'arbre reste vivant, il peut surveiller la sécurité en parallèle),
  - quand c'est fini : on applique les EFFETS sur le blackboard + SUCCESS.

Quand on branchera le réel, `initialise()` lancera ta séquence
(roby_tool_pickup, DLS, ou publiera sur /gripper) et `update()` renverra RUNNING
jusqu'à ce que le mouvement soit terminé, puis SUCCESS/FAILURE.
"""
import py_trees
from py_trees.common import Status

# Nombre de ticks simulés = "durée" d'un mouvement fictif (à ~2 Hz → ~1.5 s)
SIM_TICKS = 3


class _SimAction(py_trees.behaviour.Behaviour):
    """Base : action simulée qui dure SIM_TICKS ticks puis applique ses effets."""
    def __init__(self, name, rw_keys):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=name)
        for k in rw_keys:
            self.bb.register_key(key=k, access=py_trees.common.Access.WRITE)
        self._left = 0

    def initialise(self):
        """Appelé une fois quand l'action démarre (passe de non-RUNNING à RUNNING)."""
        self._left = SIM_TICKS
        print(f"   ▶ [SIM] {self.name} : démarrage…")

    def update(self):
        self._left -= 1
        if self._left > 0:
            return Status.RUNNING          # mouvement en cours
        self.effects()                     # mouvement terminé → on applique les effets
        print(f"   ✔ [SIM] {self.name} : terminé")
        return Status.SUCCESS

    def effects(self):
        """À surcharger : met à jour l'état cru après le mouvement."""
        raise NotImplementedError


class OpenGripper(_SimAction):
    def __init__(self):
        super().__init__(name="ouvrir la pince", rw_keys=["gripper"])

    def effects(self):
        self.bb.gripper = "open"


class CloseGripperOnObject(_SimAction):
    def __init__(self):
        super().__init__(name="fermer la pince sur l'objet", rw_keys=["gripper", "holding"])

    def effects(self):
        self.bb.gripper = "closed"
        self.bb.holding = True


class PickHead(_SimAction):
    """Aller chercher la tête `which` au rack et la monter."""
    def __init__(self, which):
        super().__init__(name=f"prendre la tête {which}", rw_keys=["head", "rack"])
        self.which = which

    def effects(self):
        self.bb.head = self.which
        rack = dict(self.bb.rack)          # copie → réassignation propre sur le blackboard
        rack[self.which] = "empty"
        self.bb.rack = rack


class ReleaseObject(_SimAction):
    """Relâcher l'objet : ouvrir la pince et oublier qu'on tenait quelque chose."""
    def __init__(self):
        super().__init__(name="relâcher l'objet", rw_keys=["gripper", "holding"])

    def effects(self):
        self.bb.gripper = "open"
        self.bb.holding = False


class PlaceHead(_SimAction):
    """Reposer au rack la tête actuellement montée (quelle qu'elle soit)."""
    def __init__(self):
        super().__init__(name="reposer la tête au rack", rw_keys=["head", "rack"])

    def effects(self):
        which = self.bb.head                # la tête montée au moment de poser
        rack = dict(self.bb.rack)
        rack[which] = "present"             # elle redevient dispo au rack
        self.bb.rack = rack
        self.bb.head = None                 # plus rien de monté
