"""
Checks LECTURE SEULE : lisent le snapshot du blackboard et jugent l'état.

Aucun ne lance ni ne tue quoi que ce soit — 100 % sûr, on peut le ticker en
boucle sans risque. Le tri-état des services :

    ABSENT             présence KO (aucun node attendu vu)
    PRÉSENT-MALSAIN    présence OK mais santé KO (topic ne circule plus)  ← le cas piège
    SAIN               présence OK et santé OK

Un service "non possédé" (lancé par décision) malsain/absent est SIGNALÉ mais ne
fait pas échouer le diagnostic global (on ne gère pas son cycle de vie).
"""
import py_trees
from py_trees.common import Status

# États de santé d'un service
ABSENT = "ABSENT"
UNHEALTHY = "PRÉSENT-MALSAIN"
HEALTHY = "SAIN"


def classify(svc, snap):
    """Renvoie (état, détail) d'un service à partir du snapshot."""
    # --- présence ---
    present = True
    if svc["nodes"]:
        present = all(n in snap["nodes"] for n in svc["nodes"])
    else:
        # pas de node fiable → on se rabat sur le live check pour juger la présence
        live = svc.get("live")
        if live:
            topic = live["topic"]
            if live["kind"] == "flow":
                present = topic in snap["flow"]
            else:
                pub, sub = snap["counts"].get(topic, (0, 0))
                present = (pub + sub) > 0
    if not present:
        return ABSENT, "aucun node/topic attendu vu"

    # --- santé ---
    live = svc.get("live")
    if not live:
        return HEALTHY, "présent (pas de sonde santé)"

    topic, kind = live["topic"], live["kind"]
    if kind == "flow":
        if snap["flow"].get(topic, False):
            return HEALTHY, f"{topic} circule"
        return UNHEALTHY, f"{topic} muet (aucun message)"
    else:  # pub / sub
        pub, sub = snap["counts"].get(topic, (0, 0))
        n = pub if kind == "pub" else sub
        if n >= 1:
            return HEALTHY, f"{topic} : {n} {kind}"
        return UNHEALTHY, f"{topic} : 0 {kind}"


class ServiceHealth(py_trees.behaviour.Behaviour):
    """
    Diagnostique UN service et imprime sa ligne de rapport.
    SUCCESS si SAIN ; FAILURE sinon. Pour un service non possédé, on renvoie
    SUCCESS quand même (on signale mais on ne le compte pas comme bloquant).
    """
    def __init__(self, svc):
        super().__init__(name=f"check {svc['name']}")
        self.svc = svc
        self.bb = self.attach_blackboard_client(name=f"check_{svc['name']}")
        self.bb.register_key(key="snap", access=py_trees.common.Access.READ)

    def update(self):
        svc = self.svc
        state, detail = classify(svc, self.bb.snap)
        icon = {HEALTHY: "✅", UNHEALTHY: "⚠️ ", ABSENT: "❌"}[state]
        tags = []
        tags.append("critique" if svc["critical"] else "optionnel")
        tags.append("possédé" if svc["owned"] else "par-décision")
        host = "pi5" if svc["host"] != "local" else "pc"
        print(f"   {icon} {svc['name']:<24} [{host:<3}] {state:<16} {detail}   ({', '.join(tags)})")

        if state == HEALTHY:
            return Status.SUCCESS
        # non possédé => on signale sans bloquer le diagnostic global
        return Status.SUCCESS if not svc["owned"] else Status.FAILURE


class InfraCheck(py_trees.behaviour.Behaviour):
    """Vérifie une précondition infra (ping / horloge / DDS) depuis le snapshot."""
    def __init__(self, kind):
        super().__init__(name=f"infra {kind}")
        self.kind = kind
        self.bb = self.attach_blackboard_client(name=f"infra_{kind}")
        self.bb.register_key(key="snap", access=py_trees.common.Access.READ)

    def update(self):
        from . import stack_spec as S
        snap = self.bb.snap
        if self.kind == "ping_pi5":
            ok = snap["ping_pi5"]
            print(f"   {'✅' if ok else '❌'} Pi5 joignable ({S.INFRA['pi5_ip']})")
        elif self.kind == "clock":
            skew = snap["clock_skew"]
            ok = skew is not None and skew <= S.INFRA["clock_skew_tol_s"]
            txt = "Pi5 injoignable" if skew is None else f"décalage {skew}s (tol {S.INFRA['clock_skew_tol_s']}s)"
            print(f"   {'✅' if ok else '❌'} horloge PC<->Pi5 : {txt}")
        elif self.kind == "dds":
            ok = snap["dds_ok"]
            print(f"   {'✅' if ok else '❌'} DDS PC pointe {S.INFRA['pc_dds_iface']}")
        else:
            ok = False
        return Status.SUCCESS if ok else Status.FAILURE
