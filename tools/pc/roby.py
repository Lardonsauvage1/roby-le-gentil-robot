#!/usr/bin/env python3
"""roby — point d'entree unique du robot. Lancer par le lanceur `roby` (environnement + mode).

    roby status  [--sim]                      toutes les portes, sans rien toucher
    roby up      --nid-confirme [--dry]       lance tout (vrai robot, tete AU NID)
    roby up      --sim [--sans-rviz]          meme chaine en simulation (domaine 43)
    roby down    [--sim] [--hors-nid]         arrete tout
    roby sortie  [--sim] [--go]               nid -> sortie (sans --go : affiche seulement)
    roby rentrer [--sim] [--go]               sortie -> nid
    roby jog     [--sim]                      panneau de jog fin
    roby collecte                             panneau de collecte du dataset (vrai robot)
    roby scene   [--sim] [--scene NOM]        (re)charge la scene de collision

C'est la procedure de /roby-lancer-bras, en code : chaque etape qui agit est precedee de portes
(roby_gates) qui arretent tout si l'etat est faux, et TOUTES les verifications prealables passent
avant la moindre action. Les skills appellent cette commande au lieu de recopier du bash.
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from collections.abc import Callable

import roby_gates as g

ICI = os.path.dirname(os.path.realpath(__file__))
MODE = g.mode_demande()
SIM = MODE == "sim"
LIBELLE = "SIMULATION" if SIM else "VRAI robot"

PI = "roby@192.168.2.37"
PI_ENV = (
    "source /opt/ros/jazzy/setup.bash; source ~/rlgr/install/setup.bash; export ROS_DOMAIN_ID=42; "
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml"
)
# Motifs en [x]yz : pgrep -f ne trouve pas le shell distant qui execute la commande (sinon il
# se tue lui-meme avant d'atteindre le Pi, vecu 2026-09-12). Liste = /roby-lancer-bras etape 0.
PI_STACK = (
    "[r]os2_control_node|[m]ove_group|[r]obot_state_publisher|[r]os2 launch roby|[h]ead_lock_node|"
    "[g]ripper_node|[l]eader_teleop|[w]rist_bldc_node|[w]rist_cli|[w]rist_bench"
)
PI_CAMERAS = "[c]am_pub"
# Cote PC, on lit /proc directement (pas d'auto-match possible) et on ne touche QU'AUX processus
# du domaine du mode : `roby down --sim` ne peut pas arreter la vraie stack, et inversement.
PC_MOTIFS = re.compile(
    r"ros2_control_node|move_group|robot_state_publisher|rviz2|pc_moveit\.launch|robot_control\.launch|"
    r"controller_manager/spawner|static_transform_publisher|scene_loader|roby_fine_jog|roby_oracle|"
    r"roby_infer|roby_guard|leader_teleop|roby_leader_panel|ros2 launch neuro|roby_collect_panel|"
    r"roby_sim_servos|bag_video_server"
)
LOG = {
    "moveit": "/tmp/roby_sim_moveit.log" if SIM else "/tmp/pc_moveit.log",
    "control": "/tmp/roby_sim_control.log",
    "servos": "/tmp/roby_sim_servos.log",
    "jog": "/tmp/roby_sim_jog.log" if SIM else "/tmp/fine_jog.log",
    "collecte": "/tmp/collect_panel.log",
}


# --------------------------------------------------------------------------------------------
# Affichage, ROS, processus
# --------------------------------------------------------------------------------------------


def afficher(resultats: list[g.Resultat]) -> bool:
    for r in resultats:
        print(f"  {'✅' if r.ok else '❌'} {r.nom:<30} {r.message}")
    return all(r.ok for r in resultats)


def etape(titre: str) -> None:
    print(f"\n── {titre}")


_NOEUD = None


def noeud():
    global _NOEUD
    if _NOEUD is None:
        import rclpy

        rclpy.init()
        _NOEUD = rclpy.create_node("roby_cli")
    return _NOEUD


def ssh(cmd: str, timeout: float = 20.0) -> tuple[int, str]:
    try:
        r = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", PI, cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return r.returncode, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 255, "ssh : delai depasse"


def pi_compte(motifs: str) -> int | None:
    """Nombre de processus du Pi5 correspondant aux motifs ; None si le Pi5 ne repond pas."""
    rc, out = ssh(f"pgrep -f '{motifs}' | wc -l")
    return int(out) if rc == 0 and out.isdigit() else None


def pi_balayer(motifs: str) -> bool:
    """kill -9 -> verifie -> recommence (un ssh en code 255 peut ne pas avoir execute le kill)."""
    for _ in range(3):
        ssh(f"for p in $(pgrep -f '{motifs}'); do kill -9 $p 2>/dev/null; done")
        time.sleep(1)
        if pi_compte(motifs) == 0:
            return True
    return False


def _ancetres() -> set[int]:
    pids, pid = set(), os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def pc_processus() -> list[tuple[int, str]]:
    """Processus ROS du PC sur le domaine du mode (lu dans leur environnement, pas le notre)."""
    domaine = f"ROS_DOMAIN_ID={g.DOMAINE[MODE]}".encode()
    moi, trouves = _ancetres(), []
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in moi:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            with open(f"/proc/{d}/environ", "rb") as f:
                env = f.read().split(b"\0")
        except OSError:
            continue
        if cmd and PC_MOTIFS.search(cmd) and domaine in env:
            trouves.append((int(d), cmd))
    return trouves


def pc_balayer() -> bool:
    for sig, attente in [(signal.SIGTERM, 4.0), (signal.SIGKILL, 2.0)]:
        for pid, _ in pc_processus():
            try:
                os.kill(pid, sig)
            except OSError:
                pass
        t0 = time.monotonic()
        while pc_processus() and time.monotonic() - t0 < attente:
            time.sleep(0.2)
    reste = pc_processus()
    for pid, cmd in reste:
        print(f"     reste : {pid} {cmd[:100]}")
    return not reste


def demarrer(cmd: list[str], log: str) -> subprocess.Popen:
    """Processus detache (son propre groupe) : il survit a `roby`."""
    env = dict(os.environ)
    env.setdefault("DISPLAY", ":0")
    return subprocess.Popen(
        cmd, stdout=open(log, "w"), stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, env=env, start_new_session=True
    )


def attendre(condition, delai: float, pas: float = 1.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < delai:
        if condition():
            return True
        time.sleep(pas)
    return False


def reseau() -> g.Resultat:
    """L'interface DDS du PC (cyclone_config.xml) doit etre montee avec une IPv4 : cable, pas de repli."""
    try:
        with open(os.path.expanduser("~/cyclone_config.xml")) as f:
            noms = re.findall(r'NetworkInterface\s+name="([^"]+)"', f.read())
    except OSError:
        return g.Resultat("reseau", False, "~/cyclone_config.xml illisible")
    for nom in noms:
        r = subprocess.run(["ip", "-br", "-4", "addr", "show", nom], capture_output=True, text=True)
        if " UP " not in f" {r.stdout} " or not re.search(r"\d+\.\d+\.\d+\.\d+", r.stdout):
            return g.Resultat(
                "reseau",
                False,
                f"{nom} n'est pas monte avec une IPv4 (cable ?) : {r.stdout.strip() or r.stderr.strip()}",
            )
    return g.Resultat("reseau", True, ", ".join(noms) or "interface auto")


# --------------------------------------------------------------------------------------------
# Portes composees
# --------------------------------------------------------------------------------------------


def portes_completes(scene: str) -> list[g.Resultat]:
    """Toutes les portes, en lecture seule (celles de `roby status`)."""
    node = noeud()
    sonde = g.Sonde(node)
    res, q, nid = g.porte_stack(node)
    if q and nid:
        au_nid = g.verifier_pose(q, nid, "pose", g.TOL_NID_DEG).ok
        res.append(g.Resultat("pose", True, f"{g.formater_pose(q)} deg ({'au nid' if au_nid else 'HORS du nid'})"))
        valide = sonde.pose_valide(q)
        if valide is None:
            res.append(g.Resultat("validite", False, "move_group ne repond pas (/check_state_validity)"))
        else:
            res.append(g.Resultat("validite", valide, "pose hors collision" if valide else "pose EN COLLISION"))
    res.append(g.porte_scene(node, scene))
    if not SIM:
        res.append(g.verifier_cameras(sonde.cameras_hz()))
    return res


def etat_actuel() -> tuple[bool, int, dict[str, float], dict[str, float]]:
    distant = False
    if not SIM:
        n = pi_compte("[r]os2_control_node")
        distant = n is None or n > 0  # Pi5 muet : on suppose que la stack tourne (prudence)
    return g.Sonde(noeud()).etat_stack(processus_distant=distant)


# --------------------------------------------------------------------------------------------
# Commandes
# --------------------------------------------------------------------------------------------


def cmd_status(a) -> int:
    print(f"Roby — {LIBELLE} (domaine {os.environ.get('ROS_DOMAIN_ID')})")
    ok = afficher(portes_completes(a.scene))
    print("→ tout est vert." if ok else "→ ÉTAT INCORRECT : ne rien faire bouger avant d'avoir corrigé.")
    return 0 if ok else 1


def cmd_up(a) -> int:
    print(f"Roby — lancement {LIBELLE} (domaine {os.environ.get('ROS_DOMAIN_ID')})")

    etape("0. Vérifications préalables (rien n'est encore touché)")
    pre = [g.verifier_domaine(MODE, os.environ.get("ROS_DOMAIN_ID"))]
    if not SIM:
        pre.append(
            g.Resultat(
                "tete au nid",
                a.nid_confirme,
                (
                    "confirmée par l'opérateur"
                    if a.nid_confirme
                    else "NON confirmée : aucun logiciel ne peut le vérifier (open-loop). Poser la tête dans le "
                    "nid, puis `roby up --nid-confirme`."
                ),
            )
        )
        pre.append(reseau())
        n = pi_compte("[r]os2_control_node")
        pre.append(g.Resultat("Pi5", n is not None, "joignable" if n is not None else f"injoignable ({PI})"))
        tourne, n_js, q, nid = etat_actuel()
        pre.append(g.decision_relance(tourne, n_js, q, nid))
    if not afficher(pre):
        print("→ REFUS : rien n'a été arrêté ni lancé.")
        return 1
    if a.dry:
        print("→ --dry : vérifications préalables vertes, rien n'a été arrêté ni lancé.")
        return 0

    etape("1. Balayage des restes (processus du domaine %s)" % g.DOMAINE[MODE])
    balayage = [g.Resultat("PC", pc_balayer(), "propre")]
    if not SIM:
        balayage.append(g.Resultat("Pi5", pi_balayer(PI_STACK), "propre"))
    if not afficher(balayage):
        return 1
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)

    etape("2. Temps réel " + ("(PC, matériel simulé)" if SIM else "(Pi5, vrais moteurs)"))
    if SIM:
        demarrer(["ros2", "launch", "roby_hardware", "robot_control.launch.py", "use_mock:=true"], LOG["control"])
        demarrer(["/usr/bin/python3", os.path.join(ICI, "roby_sim_servos.py")], LOG["servos"])
    else:
        rc, out = ssh(
            f"rm -f ~/ctrl_stack.log; setsid bash -lc '{PI_ENV}; ros2 launch roby_hardware robot_control.launch.py "
            "> ~/ctrl_stack.log 2>&1 < /dev/null &'"
        )
        if rc != 0:
            print(f"  ❌ lancement Pi5 : {out}")
            return 1
    sonde = g.Sonde(noeud())
    if not attendre(lambda: g.verifier_controleurs(sonde.controleurs(timeout=2.0)).ok, 60.0):
        afficher([g.verifier_controleurs(sonde.controleurs())])
        print("  journal : " + (LOG["control"] if SIM else f"{PI}:~/ctrl_stack.log"))
        return 1
    res, q, nid = g.porte_stack(noeud())
    if q and nid:
        res.append(g.verifier_pose(q, nid, "pose = nid", g.TOL_NID_DEG))
    if not afficher(res):
        print("→ ARRÊT : état incorrect juste après le lancement (course au mock ? fantôme ?). `roby down`.")
        return 1

    if not SIM:
        etape("3. Caméras (Pi5)")
        if pi_compte("[c]am_pub_pi2_dual") == 0:
            ssh("setsid bash -lc 'bash ~/launch_cams.sh >> ~/cams_boot.log 2>&1 < /dev/null &'")
        hz: dict[str, float] = {}

        def cameras_ok() -> bool:
            hz.update(sonde.cameras_hz(duree=2.0))
            return g.verifier_cameras(hz).ok

        attendre(cameras_ok, 30.0, pas=0.5)
        if not afficher([g.verifier_cameras(hz)]):
            print("  journal : roby@192.168.2.37:~/dual_node.log")
            return 1

    etape("4. MoveIt + scène de collision (PC)")
    demarrer(
        [
            "ros2",
            "launch",
            "neuroneimitationcarote_moveit_config",
            "pc_moveit.launch.py",
            f"scene:={a.scene}",
            f"rviz:={'false' if a.sans_rviz else 'true'}",
        ],
        LOG["moveit"],
    )

    def planif_prete() -> bool:
        try:
            return "You can start planning now" in open(LOG["moveit"]).read()
        except OSError:
            return False

    if not attendre(planif_prete, 90.0):
        print(f"  ❌ move_group n'est pas prêt (journal {LOG['moveit']})")
        return 1
    attendre(lambda: g.porte_scene(noeud(), a.scene).ok, 90.0, pas=2.0)

    etape("5. Contrôle final")
    ok = afficher(portes_completes(a.scene))
    if ok:
        print(
            "→ stack prête, bras au nid, rien n'a bougé."
            + ("" if SIM else " Suite : `roby sortie` (affiche), puis `roby sortie --go`.")
        )
    return 0 if ok else 1


def cmd_down(a) -> int:
    print(f"Roby — arrêt {LIBELLE} (domaine {os.environ.get('ROS_DOMAIN_ID')})")
    if not SIM:
        tourne, n_js, q, nid = etat_actuel()
        if not afficher([g.decision_arret(tourne, n_js, q, nid, a.hors_nid)]):
            print("→ REFUS : rien n'a été arrêté.")
            return 1
    ok = [g.Resultat("PC", pc_balayer(), "arrêté")]
    if not SIM:
        ok.append(g.Resultat("Pi5 stack", pi_balayer(PI_STACK), "arrêtée"))
        ok.append(g.Resultat("Pi5 caméras", pi_balayer(PI_CAMERAS), "arrêtées"))
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    return 0 if afficher(ok) else 1


def cmd_sortie(a, reverse: bool = False) -> int:
    cmd = (
        ["bash", os.path.join(ICI, "roby_sortie_nid.sh")]
        + (["--reverse"] if reverse else [])
        + (["--go"] if a.go else [])
    )
    rc = subprocess.call(cmd)
    if rc != 0 or not a.go or reverse:
        return rc
    # Etat de travail apres la sortie (/roby-lancer-bras 4.z) : tete re-verrouillee (elle ne tient
    # plus l'outil sinon), pince ouverte. Le verrou alimente la pince : verrouiller d'abord.
    from std_msgs.msg import Bool

    node = noeud()
    for topic, valeur, attente in [("/head_lock", True, 1.5), ("/gripper", False, 0.5)]:
        pub = node.create_publisher(Bool, topic, 10)
        attendre(lambda: pub.get_subscription_count() > 0, 5.0, pas=0.1)
        pub.publish(Bool(data=valeur))
        time.sleep(attente)
    print("tête re-verrouillée, pince ouverte (état de travail).")
    return 0


def cmd_panneau(script: str, log: str, scene: bool = False) -> int:
    # Porte ICI, dans le terminal : celle du programme (defense en profondeur) ecrirait son
    # refus dans un journal qu'on ne lit pas.
    g.exiger(noeud(), script, scene=scene)
    p = demarrer(["bash", os.path.join(ICI, script)], log)
    time.sleep(4)
    if p.poll() is not None:
        print(f"❌ {script} s'est arrêté (code {p.returncode}) — journal {log} :")
        print(open(log).read()[-1500:])
        return 1
    print(f"✅ {script} ouvert (journal {log})")
    return 0


def cmd_scene(a) -> int:
    r = subprocess.run(
        ["ros2", "run", "roby_environments", "scene_loader", "--env", a.scene], capture_output=True, text=True
    )
    if r.returncode != 0:
        print(f"❌ chargement de la scène '{a.scene}' échoué (move_group tourne-t-il ?)")
        return 1
    return 0 if afficher([g.porte_scene(noeud(), a.scene)]) else 1


def main() -> int:
    ap = argparse.ArgumentParser(prog="roby", description="Point d'entrée unique du robot Roby.")
    ap.add_argument("--sim", action="store_true", help="simulation (domaine 43) — lu par le lanceur `roby`")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def commande(nom: str, aide: str) -> argparse.ArgumentParser:
        p = sub.add_parser(nom, help=aide)
        p.add_argument("--sim", action="store_true", help="simulation (domaine 43)")
        return p

    for nom, aide in [("status", "toutes les portes, sans rien toucher"), ("scene", "(re)charger la scène")]:
        commande(nom, aide).add_argument("--scene", default=g.SCENE_DEFAUT)
    p = commande("up", "lancer toute la stack")
    p.add_argument("--nid-confirme", action="store_true", help="la tête est posée dans le nid (vrai robot)")
    p.add_argument("--dry", action="store_true", help="vérifications préalables seulement")
    p.add_argument("--sans-rviz", action="store_true")
    p.add_argument("--scene", default=g.SCENE_DEFAUT)
    commande("down", "tout arrêter").add_argument(
        "--hors-nid", action="store_true", help="arrêter bras hors du nid (assumé)"
    )
    for nom in ("sortie", "rentrer"):
        commande(nom, f"{'nid -> sortie' if nom == 'sortie' else 'sortie -> nid'}").add_argument(
            "--go", action="store_true", help="BOUGER (sans : affiche la trajectoire)"
        )
    commande("jog", "panneau de jog fin")
    commande("collecte", "panneau de collecte du dataset")

    a = ap.parse_args()
    # Meme regle que le lanceur bash (--sim n'importe ou sur la ligne) : l'attribut a.sim ne suffit
    # pas, argparse l'ecrase par le defaut du sous-parseur dans `roby --sim status`.
    if ("--sim" in sys.argv[1:]) != SIM:
        print("❌ passer par le lanceur `roby` : c'est lui qui fixe le domaine selon --sim.", file=sys.stderr)
        return 2
    if a.cmd == "collecte" and SIM:
        print("❌ pas de collecte en simulation : pas de caméras, les épisodes seraient sans images.", file=sys.stderr)
        return 2
    actions: dict[str, Callable[[argparse.Namespace], int]] = {
        "status": cmd_status,
        "up": cmd_up,
        "down": cmd_down,
        "sortie": cmd_sortie,
        "rentrer": lambda a: cmd_sortie(a, reverse=True),
        "jog": lambda a: cmd_panneau("roby_fine_jog.sh", LOG["jog"]),
        "collecte": lambda a: cmd_panneau("roby_collect_panel.sh", LOG["collecte"], scene=True),
        "scene": cmd_scene,
    }
    return actions[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
