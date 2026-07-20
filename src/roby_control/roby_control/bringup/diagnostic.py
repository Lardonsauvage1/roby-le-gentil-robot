"""
Arbre de DIAGNOSTIC (lecture seule) : photographie la stack et imprime un
rapport d'état, sans rien lancer ni tuer. C'est l'étape 1 du superviseur —
sûre à ticker en boucle. Les réparations (launch/kill) viendront ensuite.

Structure :
    Sequence "diagnostic"
    ├── Snapshot                      (prend la photo -> blackboard)
    ├── Parallel "infra"              (ping / horloge / DDS — tous évalués)
    └── Parallel "services"           (chaque service jugé : ABSENT/MALSAIN/SAIN)

On utilise Parallel (et pas Selector) pour FORCER l'évaluation de TOUS les
enfants à chaque tick : un diagnostic doit tout regarder, pas s'arrêter au 1er KO.

Usage (après avoir sourcé ROS) :
    python3 -m roby_control.bringup.diagnostic          # un passage
    python3 -m roby_control.bringup.diagnostic watch    # en boucle toutes les 5 s
"""
import sys
import time
import py_trees
from py_trees.common import Status, ParallelPolicy
from . import stack_spec as S
from .sense import Snapshot
from .checks import ServiceHealth, InfraCheck


def build_tree():
    root = py_trees.composites.Sequence(name="diagnostic", memory=True)

    infra = py_trees.composites.Parallel(
        name="infra", policy=ParallelPolicy.SuccessOnAll(synchronise=False))
    infra.add_children([InfraCheck("ping_pi5"), InfraCheck("clock"), InfraCheck("dds")])

    services = py_trees.composites.Parallel(
        name="services", policy=ParallelPolicy.SuccessOnAll(synchronise=False))
    services.add_children([ServiceHealth(svc) for svc in S.STACK])

    root.add_children([Snapshot(), infra, services])
    return root


def run_once():
    root = build_tree()
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    print("\n=== DIAGNOSTIC STACK ROBY ===")
    print("-- infra --")
    # On tick jusqu'à ce que le Snapshot + checks aient tourné (une passe suffit,
    # tout est synchrone sauf les commandes shell).
    tree.tick()
    # Les prints des checks sortent pendant le tick ; on imprime le verdict après.

    infra_ok = all(c.status == Status.SUCCESS
                   for c in root.children[1].children)
    crit_fail = [s.svc["name"] for s in root.children[2].children
                 if s.status == Status.FAILURE]  # que des possédés-critiques/possédés

    print("\n-- verdict --")
    if not infra_ok:
        print("❌ INFRA incomplète — corriger avant de juger la stack.")
    if crit_fail:
        print(f"❌ Services possédés non sains : {', '.join(crit_fail)}")
    if infra_ok and not crit_fail:
        print("✅ STACK SAINE (tous les services possédés sont OK).")
    print("   (⚠️ = présent mais malsain ; les 'par-décision' sont signalés, non bloquants)\n")
    return 0 if (infra_ok and not crit_fail) else 1


def main():
    args = sys.argv[1:]
    if args and args[0] == "watch":
        try:
            while True:
                run_once()
                time.sleep(5)
        except KeyboardInterrupt:
            return 0
    return run_once()


if __name__ == "__main__":
    sys.exit(main())
