"""
Réparations IDEMPOTENTES (étape 2). Ici on AGIT enfin : tuer / (re)lancer.

Motif idempotent d'une réparation :
    check santé → si SAIN : ne rien faire
                → sinon : tuer les restes éventuels → lancer → re-vérifier

Encapsulé dans un Fallback :
    Fallback "assurer <svc>"
    ├── ServiceHealth(<svc>)   ← déjà sain ? SUCCESS, on ne touche à rien
    └── Repair(<svc>)          ← sinon : kill stale + launch + attente

⚠️ GARDE-FOUS (spec) :
  - `owned:False` (lancé par décision) → JAMAIS tué/relancé ici (Repair refuse).
  - Un service critique déjà PRÉSENT mais malsain n'est pas brutalement relancé
    sans qu'on l'ait décidé (cf « ne pas activer les contrôleurs à la main »).
    rviz est le cobaye : possédé, non critique, jetable, PC-local, zéro moteur.
"""
import os
import time
import signal
import subprocess
import py_trees
from py_trees.common import Status
from . import stack_spec as S
from .sense import sh, ros_nodes
from .checks import ServiceHealth

# ⚠️ Garde-fou moteur : tant que False, Repair REFUSE de (re)lancer un service
# `moves_motors` (activer les contrôleurs = risque d'à-coup, cf BUG-006).
# bringup le passe à True seulement avec le drapeau --go-motors.
ALLOW_MOTORS = False

# Sourcing complet pour un shell ssh non-interactif sur le Pi5 (le .bashrc ne
# source PAS le workspace rlgr).
PI5_SOURCE = ("source /opt/ros/jazzy/setup.bash; "
              "source ~/rlgr/install/setup.bash; "
              "export ROS_DOMAIN_ID=42; "
              "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
              "export CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml")


def _pids_matching(pattern):
    """PIDs dont la ligne de commande contient `pattern` (hors nous-mêmes)."""
    rc, out = sh(f"pgrep -f '{pattern}'", timeout=4)
    pids = []
    for line in out.splitlines():
        line = line.strip()
        if line.isdigit() and int(line) != os.getpid():
            pids.append(int(line))
    return pids


def kill_stale(svc):
    """Tue les restes de process d'un service (TERM puis KILL). Idempotent : no-op si rien."""
    killed = []
    for pat in svc.get("proc_match", []):
        for pid in _pids_matching(pat):
            try:
                os.kill(pid, signal.SIGTERM)
                killed.append(pid)
            except ProcessLookupError:
                pass
    if not killed:
        return []
    time.sleep(2.0)  # laisse le temps de sortir proprement
    for pid in killed:
        try:
            os.kill(pid, signal.SIGKILL)  # achève les récalcitrants
        except ProcessLookupError:
            pass
    return killed


def launch_local(svc, log_dir="/tmp"):
    """Lance un service PC en détaché (session propre, logs fichier)."""
    env = dict(os.environ)
    env.pop("GTK_PATH", None)  # piège connu : GTK_PATH (env snap VS Code) fait crasher rviz
    log = os.path.join(log_dir, f"roby_{svc['name']}.log")
    logf = open(log, "w")
    # bash -lc pour hériter d'un environnement ROS complet ; start_new_session
    # => le service survit à la fin du runner (pas dans notre groupe de process).
    subprocess.Popen(["bash", "-lc", f"exec {svc['launch']}"],
                     stdout=logf, stderr=subprocess.STDOUT,
                     start_new_session=True, env=env)
    return log


def kill_over_ssh(svc):
    """Tue les restes d'un service Pi5 via ssh (pgrep+kill par PID, sans le piège
    pkill-self : on exclut le shell courant $$ et son parent $PPID)."""
    host = svc["host"]
    pats = svc.get("proc_match", [])
    if not pats:
        return []
    pat = "|".join(pats)
    # pgrep matche aussi le bash qui porte le motif dans sa ligne → on l'exclut.
    loop = (f'for p in $(pgrep -f "{pat}" | grep -vw $$ | grep -vw $PPID); '
            f'do kill -{{sig}} $p 2>/dev/null; done')
    sh(f"ssh -o ConnectTimeout=6 roby@{host} '{loop.format(sig='TERM')}'", timeout=10)
    time.sleep(2.0)
    sh(f"ssh -o ConnectTimeout=6 roby@{host} '{loop.format(sig='KILL')}'", timeout=10)
    return pats


def launch_over_ssh(svc):
    """Lance un service Pi5 détaché via ssh (setsid, source rlgr, log distant)."""
    host = svc["host"]
    log = f"/tmp/roby_{svc['name']}.log"
    inner = (f'setsid bash -lc "{PI5_SOURCE}; exec {svc["launch"]}" '
             f'>{log} 2>&1 </dev/null &')
    sh(f"ssh -o ConnectTimeout=6 roby@{host} '{inner}'", timeout=15)
    return f"{host}:{log}"


class Repair(py_trees.behaviour.Behaviour):
    """
    Action idempotente : (kill stale) → launch → attend que le node apparaisse.
    RUNNING pendant l'attente, SUCCESS si présent, FAILURE si timeout/refus.
    Local (PC) ou distant (Pi5 via ssh) selon svc['host'].
    """
    def __init__(self, svc):
        super().__init__(name=f"réparer {svc['name']}")
        self.svc = svc
        self._deadline = None
        self._log = None

    def initialise(self):
        svc = self.svc
        self._deadline = None
        # --- garde-fous ---
        if not svc["owned"]:
            print(f"   ⛔ {svc['name']} 'par-décision' → on NE répare PAS (signalé seulement)")
            return
        if svc.get("moves_motors") and not ALLOW_MOTORS:
            print(f"   🛑 {svc['name']} activerait les contrôleurs (à-coup moteur possible).")
            print(f"      Refusé sans --go-motors ET ta présence près du robot.")
            return
        # --- kill + launch (local ou ssh) ---
        remote = svc["host"] != "local"
        killed = kill_over_ssh(svc) if remote else kill_stale(svc)
        print(f"   🧹 restes tués : {killed if killed else 'aucun'}")
        self._log = launch_over_ssh(svc) if remote else launch_local(svc)
        print(f"   🚀 lancé : {svc['launch']}  (log: {self._log})")
        # le groupe Pi5 (spawners à 6s) met plus longtemps à apparaître
        self._deadline = time.time() + (40.0 if remote else 25.0)

    def update(self):
        if self._deadline is None:
            return Status.FAILURE  # refusé (par-décision, moteur bloqué)
        present = all(n in ros_nodes() for n in self.svc["nodes"])
        if present:
            print(f"   ✅ {self.svc['name']} de retour")
            return Status.SUCCESS
        if time.time() > self._deadline:
            print(f"   ❌ {self.svc['name']} pas revenu à temps")
            return Status.FAILURE
        return Status.RUNNING


def ensure(svc):
    """Fallback 'déjà sain ? SINON répare' pour un service."""
    fb = py_trees.composites.Selector(name=f"assurer {svc['name']}", memory=False)
    fb.add_children([ServiceHealth(svc), Repair(svc)])
    return fb


# ---------------- runner ----------------
def _find(name):
    for svc in S.STACK:
        if svc["name"] == name:
            return svc
    return None


def main():
    import sys
    from .sense import Snapshot
    args = sys.argv[1:]
    if not args:
        print("usage: python3 -m roby_control.bringup.repair <service>")
        print("services:", ", ".join(s["name"] for s in S.STACK))
        return 2
    svc = _find(args[0])
    if not svc:
        print(f"service inconnu : {args[0]}")
        return 2

    root = py_trees.composites.Sequence(name=f"assurer_{svc['name']}", memory=True)
    root.add_children([Snapshot(), ensure(svc)])
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    print(f"\n=== ASSURER '{svc['name']}' (idempotent) ===")
    tick = 0
    while True:
        tick += 1
        tree.tick()
        if root.status != Status.RUNNING:
            break
        time.sleep(1.0)

    ok = root.status == Status.SUCCESS
    print(f"\nRésultat : {'✅ service assuré' if ok else '❌ échec'}")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
