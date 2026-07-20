"""
Feuilles "CONDITION" : lecture seule, instantanées.

Une condition ne bouge RIEN. Elle lit le blackboard et renvoie :
  - SUCCESS si la condition est vraie
  - FAILURE sinon

Comme l'arbre est ré-évalué à chaque tick, ces conditions sont re-vérifiées en
continu — c'est ce qui rend le comportement "réactif". Le jour où tu ajoutes un
vrai capteur, il suffira de faire lire ce capteur ici au lieu de l'état cru.
"""
import py_trees
from py_trees.common import Status


class _Condition(py_trees.behaviour.Behaviour):
    """Base commune : attache un client blackboard en LECTURE sur les clés utiles."""
    def __init__(self, name, read_keys):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=name)
        for k in read_keys:
            self.bb.register_key(key=k, access=py_trees.common.Access.READ)


class HeadIs(_Condition):
    """SUCCESS si la tête montée est bien `expected` (ex. 'pince', ou None)."""
    def __init__(self, expected):
        super().__init__(name=f"tête == {expected} ?", read_keys=["head"])
        self.expected = expected

    def update(self):
        return Status.SUCCESS if self.bb.head == self.expected else Status.FAILURE


class HeadMounted(_Condition):
    """SUCCESS si une tête (n'importe laquelle) est montée."""
    def __init__(self):
        super().__init__(name="une tête est montée ?", read_keys=["head"])

    def update(self):
        return Status.SUCCESS if self.bb.head is not None else Status.FAILURE


class NotHolding(_Condition):
    """SUCCESS si aucun objet n'est agrippé."""
    def __init__(self):
        super().__init__(name="rien en main ?", read_keys=["holding"])

    def update(self):
        return Status.SUCCESS if not self.bb.holding else Status.FAILURE


class Holding(_Condition):
    """SUCCESS si un objet est actuellement agrippé."""
    def __init__(self):
        super().__init__(name="objet en main ?", read_keys=["holding"])

    def update(self):
        return Status.SUCCESS if self.bb.holding else Status.FAILURE


class GripperIs(_Condition):
    """SUCCESS si la pince est dans l'état `state` ('open' ou 'closed')."""
    def __init__(self, state):
        super().__init__(name=f"pince == {state} ?", read_keys=["gripper"])
        self.state = state

    def update(self):
        return Status.SUCCESS if self.bb.gripper == self.state else Status.FAILURE


class RackHas(_Condition):
    """SUCCESS si l'outil `which` est présent dans le rack."""
    def __init__(self, which):
        super().__init__(name=f"rack a {which} ?", read_keys=["rack"])
        self.which = which

    def update(self):
        return Status.SUCCESS if self.bb.rack.get(self.which) == "present" else Status.FAILURE


class AtIs(_Condition):
    """SUCCESS si le bras se croit dans la zone `zone` ('nid', 'approche_nid', 'libre')."""
    def __init__(self, zone):
        super().__init__(name=f"bras au {zone} ?", read_keys=["at"])
        self.zone = zone

    def update(self):
        return Status.SUCCESS if self.bb.at == self.zone else Status.FAILURE
