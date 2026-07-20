"""
Composition des arbres (les "skills" = compétences), motorisés.

Motif de prise/dépose (réponse Sam) : on ne fonce jamais direct sur une cible.
    libre (MoveIt) → POINT D'APPROCHE → (insertion) → CIBLE
et au retour : CIBLE → POINT D'APPROCHE → dégagé.

Composites :
  - Sequence  : enchaîne, s'arrête au 1er échec (préconditions dures PUIS mouvement).
  - Selector  : "condition ? SINON répare" (précondition souple).
"""
import py_trees
from . import conditions as C
from . import motion_actions as M
from . import positions as P
from . import world


# ---------------- nid / approche ----------------
def go_to_nest():
    """SKILL : rejoindre le nid (dock de référence) via son point d'approche."""
    root = py_trees.composites.Sequence(name="aller_au_nid", memory=True)
    root.add_children([
        M.MoveVia([P.NEST["approach"], P.NEST["dock"]], arrive_at="nid"),
    ])
    return root


def exit_nest():
    """SKILL : sortir du nid en remontant à son point d'approche."""
    root = py_trees.composites.Sequence(name="sortir_du_nid", memory=True)
    root.add_children([
        C.AtIs("nid"),                              # dur : il faut être au nid
        M.MoveVia([P.NEST["approach"]], arrive_at="approche_nid"),
    ])
    return root


# ---------------- têtes (pince / ventouse) ----------------
def pick_head(which="pince"):
    """
    SKILL : monter la tête `which` depuis son logement du rack.
    Dures : aucune tête montée + `which` disponible au rack.
    Mouvement : libre → approche → logement, verrou ON, puis dégage à l'approche.
    """
    h = P.HEADS[which]
    root = py_trees.composites.Sequence(name=f"prendre_tete_{which}", memory=True)
    root.add_children([
        C.HeadIs(None),                             # dur : pas déjà une tête
        C.RackHas(which),                           # dur : tête dispo
        M.MoveVia([h["approach"], h["rack"]], arrive_at=f"rack_{which}"),
        M.HeadLock(True),                           # verrouille la tête sur la monture
        M.SetHead(which),                           # belief : head=which, logement vidé
        M.MoveVia([h["approach"]], arrive_at="libre"),
    ])
    return root


def place_head():
    """
    SKILL : reposer la tête montée dans son logement.
    Dures : une tête montée + rien en main.
    Mouvement : libre → approche → logement, verrou OFF, puis dégage à l'approche.
    (Le logement visé dépend de la tête actuellement montée, lue dans l'état.)
    """
    which = world.load_world().get("head") or "pince"   # placeholder si None (préco échoue avant)
    h = P.HEADS[which]
    root = py_trees.composites.Sequence(name="reposer_tete", memory=True)
    root.add_children([
        C.HeadMounted(),                            # dur : il faut une tête à reposer
        C.NotHolding(),                             # dur : pas d'objet encore agrippé
        M.MoveVia([h["approach"], h["rack"]], arrive_at=f"rack_{which}"),
        M.HeadLock(False),                          # déverrouille → laisse la tête
        M.ClearHead(),                              # belief : logement rempli, head=None
        M.MoveVia([h["approach"]], arrive_at="libre"),
    ])
    return root


# ---------------- objet (pince) ----------------
def ensure_gripper_open():
    sel = py_trees.composites.Selector(name="assurer pince ouverte", memory=False)
    sel.add_children([C.GripperIs("open"), M.Grip(False)])
    return sel


def grab_object():
    """SKILL : attraper un objet (tête pince, rien en main, pince ouverte→ferme)."""
    root = py_trees.composites.Sequence(name="attraper_objet", memory=True)
    root.add_children([
        C.HeadIs("pince"),
        C.NotHolding(),
        ensure_gripper_open(),                      # souple : ouvre si fermée
        M.Grip(True),                               # ferme sur l'objet
        M.SetHolding(True),
    ])
    return root


def release_object():
    """SKILL : relâcher l'objet tenu."""
    root = py_trees.composites.Sequence(name="relacher_objet", memory=True)
    root.add_children([
        C.HeadIs("pince"),
        C.Holding(),
        M.Grip(False),                              # ouvre
        M.SetHolding(False),
    ])
    return root


# Registre : nom CLI -> fabrique d'arbre
SKILLS = {
    "go_to_nest": go_to_nest,
    "exit_nest": exit_nest,
    "pick_head": pick_head,
    "place_head": place_head,
    "grab_object": grab_object,
    "release_object": release_object,
}
