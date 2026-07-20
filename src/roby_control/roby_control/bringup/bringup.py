"""
BRINGUP COMPLET : démarre (ou complète) toute la stack d'une seule commande.

Séquence (memory=True, on ne recule pas une étape validée) :
    Snapshot                    photo initiale
    → infra (ping/clock/dds)    préconditions ; ABORT propre si KO
    → ensure(rsp)               groupe Pi5 (robot_control.launch.py) ⚠️ moteurs
    → ensure(move_group)        groupe PC (pc_moveit.launch.py : move_group + rviz)
    → Snapshot                  re-photo après les lancements
    → Report                    tableau complet ; OK ssi tous les CRITIQUES possédés sont SAINS

Idempotent : chaque `ensure()` ne (re)lance QUE si le service est absent. Lancé
sur une stack déjà saine → tout vert, aucun relancement (zéro risque).

⚠️ SÉCURITÉ MOTEUR : lancer le groupe Pi5 active arm_controller → à-coup possible
(BUG-006). Par défaut Repair REFUSE. Pour un vrai démarrage à froid :
    python3 -m roby_control.bringup.bringup --go-motors     (près du robot !)

Usage :
    python3 -m roby_control.bringup.bringup                 # complète/vérifie, sans toucher aux moteurs
    python3 -m roby_control.bringup.bringup --go-motors     # autorise le (re)lancement Pi5
"""
import sys
import time
import py_trees
from py_trees.common import Status
from . import stack_spec as S
from . import repair as R
from .sense import Snapshot
from .checks import InfraCheck, classify, HEALTHY


def _find(name):
    return next(s for s in S.STACK if s["name"] == name)


class Report(py_trees.behaviour.Behaviour):
    """Tableau final de tous les services ; SUCCESS ssi tous les critiques possédés sont SAINS."""
    def __init__(self):
        super().__init__(name="rapport final")
        self.bb = self.attach_blackboard_client(name="report")
        self.bb.register_key(key="snap", access=py_trees.common.Access.READ)

    def update(self):
        snap = self.bb.snap
        print("\n-- état final de la stack --")
        blocking = []
        for svc in S.STACK:
            state, detail = classify(svc, snap)
            icon = {HEALTHY: "✅"}.get(state, "⚠️ " if state.startswith("PRÉSENT") else "❌")
            host = "pi5" if svc["host"] != "local" else "pc"
            flag = "critique" if svc["critical"] else "optionnel"
            print(f"   {icon} {svc['name']:<24} [{host:<3}] {state:<16} {detail}   ({flag})")
            if svc["owned"] and svc["critical"] and state != HEALTHY:
                blocking.append(svc["name"])
        if blocking:
            print(f"\n❌ BRINGUP INCOMPLET — critiques non sains : {', '.join(blocking)}")
            return Status.FAILURE
        print("\n✅ STACK PRÊTE (tous les critiques possédés sont SAINS ; moteurs au repos, non commandés).")
        return Status.SUCCESS


def build_tree():
    root = py_trees.composites.Sequence(name="bringup_roby", memory=True)

    infra = py_trees.composites.Sequence(name="infra", memory=True)
    infra.add_children([InfraCheck("ping_pi5"), InfraCheck("clock"), InfraCheck("dds")])

    root.add_children([
        Snapshot(),                       # photo initiale
        infra,                            # préconditions (abort si KO)
        R.ensure(_find("rsp")),           # groupe Pi5 (gated moteur)
        R.ensure(_find("move_group")),    # groupe PC
        Snapshot(),                       # re-photo après lancements
        Report(),                         # verdict
    ])
    return root


def main():
    args = sys.argv[1:]
    if "--go-motors" in args:
        R.ALLOW_MOTORS = True
        print("⚠️  --go-motors : le (re)lancement du groupe Pi5 est AUTORISÉ (moteurs).")

    root = build_tree()
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=20)

    print("\n=== BRINGUP STACK ROBY ===")
    tick = 0
    while True:
        tick += 1
        tree.tick()
        if root.status != Status.RUNNING:
            break
        time.sleep(2.0)

    ok = root.status == Status.SUCCESS
    print(f"\nRésultat : {'✅ stack prête' if ok else '❌ bringup incomplet'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
