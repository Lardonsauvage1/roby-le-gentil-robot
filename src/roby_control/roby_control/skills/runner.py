"""
Runner : lance un skill par son nom, tick l'arbre jusqu'au bout, sauve l'état.

Usage :
  python3 -m roby_control.skills.runner <skill> [arg] [--real]
  python3 -m roby_control.skills.runner go_to_nest
  python3 -m roby_control.skills.runner pick_head ventouse
  python3 -m roby_control.skills.runner pick_head pince --real   # ⚠️ BOUGE LE BRAS
  python3 -m roby_control.skills.runner reset          # resync état cru (open-loop)
  python3 -m roby_control.skills.runner state          # affiche l'état courant
  python3 -m roby_control.skills.runner poses          # poses connues / manquantes

⚠️ DÉFAUT = SIM (rien ne bouge). --real pilote le vrai bras/servos → uniquement
avec la validation de Sam, robot surveillé.

Boucle de tick : l'arbre est ré-évalué en boucle. Une ACTION longue renvoie
RUNNING plusieurs ticks (mouvement en cours) ; on continue jusqu'à ce que la
racine ne soit plus RUNNING (SUCCESS = fait, FAILURE = refusé/échoué).
"""
import sys
import time
import py_trees
from py_trees.common import Status
from . import world
from . import trees
from . import motion
from . import positions
from .motion_actions import MoveVia

TICK_HZ = 2.0  # cadence de tick (lente en SIM pour bien voir ; ~10 Hz en réel)


ALIGN_TOL = 0.15   # rad (~8.6°) : écart max toléré croyance contrôleur vs pose attendue


def _required_poses(node, acc):
    """Collecte les poses effectivement utilisées par l'arbre (feuilles MoveVia)."""
    if isinstance(node, MoveVia):
        acc.update(node.pose_names)
    for child in getattr(node, "children", []):
        _required_poses(child, acc)


def _expected_pose_name(at):
    """Zone crue `at` -> nom de pose attendue (ou None si non vérifiable)."""
    if at in ("nid", "approche_nid"):
        return at
    if isinstance(at, str) and at.startswith("rack_"):
        h = at[len("rack_"):]
        return positions.HEADS.get(h, {}).get("rack")
    return None   # "libre" / inconnu -> pas de pose fixe à vérifier


def _preflight_alignment(state):
    """Vérifie que le contrôleur (/joint_states) est bien là où le skill le croit.
    Renvoie (ok, message). Refuse si divergence -> évite le balayage dangereux."""
    name = _expected_pose_name(state.get("at"))
    if name is None:
        return True, (f"⚠️  position de départ '{state.get('at')}' non vérifiable "
                      f"(pas de pose fixe) — VÉRIFIE À L'ŒIL que le bras y est.")
    poses = positions.load_poses()
    if name not in poses:
        return True, f"⚠️  pose '{name}' absente du YAML — vérif d'alignement sautée."
    err, believed = motion.alignment_error(poses[name])
    if err is None:
        return False, "⛔ /joint_states illisible (stack Pi5 lancée ?) — RÉEL annulé."
    if err > ALIGN_TOL:
        return False, (
            f"⛔ DÉSALIGNEMENT : le contrôleur se croit à {[round(x,3) for x in believed]}\n"
            f"   mais le skill attend '{name}' = {poses[name]} (écart {err:.2f} rad).\n"
            f"   → replace le bras au bon endroit + redémarre la stack RT pour réaligner. RÉEL annulé.")
    return True, f"✅ alignement OK : contrôleur ≈ '{name}' (écart {err:.3f} rad)."


def _run_skill(name, arg, real=False):
    if name not in trees.SKILLS:
        print(f"skill inconnu : {name}. Dispo : {', '.join(trees.SKILLS)}")
        return 2

    state = world.load_world()
    world.seed_blackboard(state)
    print(f"État initial : {world.pretty(state)}\n")

    factory = trees.SKILLS[name]
    root = factory(arg) if arg is not None else factory()

    if real:
        need = set()
        _required_poses(root, need)
        have = positions.load_poses()
        miss = sorted(p for p in need if p not in have)
        if miss:
            print(f"⛔ RÉEL annulé — poses manquantes pour ce skill : {miss}")
            return 2
        # GARDE-FOU alignement : la croyance du contrôleur doit matcher la
        # position de départ crue du skill, sinon MoveIt planifie un gros
        # déplacement faux (cf incident axe 1, 2026-07-07).
        ok, msg = _preflight_alignment(state)
        print(msg)
        if not ok:
            return 3
        print(f"⚠️  MODE RÉEL : le bras va bouger. Poses utilisées : {sorted(need)}")

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)

    period = 1.0 / TICK_HZ
    tick = 0
    while True:
        tick += 1
        tree.tick()
        print(f"--- tick {tick} ---")
        print(py_trees.display.unicode_tree(root=root, show_status=True))
        if root.status != Status.RUNNING:
            break
        time.sleep(period)

    # Persiste l'état cru mis à jour par les effets
    final = world.dump_blackboard()
    world.save_world(final)

    ok = root.status == Status.SUCCESS
    print(f"\nRésultat : {'✅ SUCCÈS' if ok else '❌ REFUS/ÉCHEC'}")
    print(f"État final : {world.pretty(final)}")
    return 0 if ok else 1


def main():
    args = sys.argv[1:]
    real = "--real" in args
    args = [a for a in args if a != "--real"]
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        return 0

    cmd = args[0]
    arg = args[1] if len(args) > 1 else None

    if cmd == "state":
        print(world.pretty(world.load_world()))
        return 0
    if cmd == "reset":
        world.save_world(dict(world.DEFAULT_STATE))
        print("État cru resynchronisé (pose nid) :", world.pretty(world.DEFAULT_STATE))
        return 0
    if cmd == "poses":
        have = positions.load_poses()
        print("Poses connues :", ", ".join(sorted(have)) or "(aucune)")
        miss = positions.missing_poses()
        print("Poses MANQUANTES (référencées mais absentes) :",
              ", ".join(miss) if miss else "aucune ✅")
        return 0

    if real:
        motion.SIM = False
    return _run_skill(cmd, arg, real=real)


if __name__ == "__main__":
    sys.exit(main())
