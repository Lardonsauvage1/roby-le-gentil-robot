"""Portes de controle de la stack Roby — UNE seule copie, lue par `roby` ET par les programmes
qui commandent le bras (jog, sortie du nid, oracle, garde...).

Une porte LIT l'etat de la stack et dit s'il est sain. Elle ne publie rien et n'appelle que des
services de lecture (GetPlanningScene, GetStateValidity) : aucune porte ne peut faire bouger le bras.

Pourquoi ce module (spec-point-entree-unique-lancement, 2026-09-13) : ces controles n'existaient
que dans le texte des skills, recopie a la main a chaque lancement. Un script lance hors procedure
ne verifiait rien — BUG-008 : le jog a recopie la pose d'un bras simule, 38 deg de saut.

Mode : `ROBY_SIM=1` => simulation (domaine 43, materiel mock), sinon vrai robot (domaine 42,
RobySystem). La source qui fait foi pour le materiel ET pour la pose du nid est le
/robot_description publie par la stack elle-meme : c'est ce que le controleur a charge.
"""

from __future__ import annotations

import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass

JOINTS = [f"joint_{i}" for i in range(1, 6)]
DOMAINE = {"reel": 42, "sim": 43}
PLUGIN = {"reel": "roby_hardware/RobySystem", "sim": "mock_components/GenericSystem"}
CAMERAS = {
    "left": "/head_camera/left/image_raw/compressed",
    "right": "/head_camera/right/image_raw/compressed",
}
CAMERA_HZ_MIN = 12.0  # nominal 15 Hz (cam_pub_pi2_dual)
TOL_NID_DEG = 0.1  # la pose lue au demarrage EST celle du nid (open-loop) : 0,1 deg suffit
SCENE_DEFAUT = "cuisine"


@dataclass
class Resultat:
    """Verdict d'une porte."""

    nom: str
    ok: bool
    message: str


# --------------------------------------------------------------------------------------------
# Logique pure (sans ROS) — testee dans test/test_roby_gates.py
# --------------------------------------------------------------------------------------------


def mode_demande(env: Mapping[str, str] | None = None) -> str:
    """'sim' ou 'reel'. ROBY_SIM (pose par `roby` et roby_ros_env.sh) fait foi ; absent — un
    script lance a la main, un banc de test — le domaine decide : 43 = simulation."""
    lu: Mapping[str, str] = os.environ if env is None else env
    roby_sim = lu.get("ROBY_SIM")
    if roby_sim in ("0", "1"):
        return "sim" if roby_sim == "1" else "reel"
    return "sim" if lu.get("ROS_DOMAIN_ID") == str(DOMAINE["sim"]) else "reel"


def verifier_domaine(mode: str, domaine_env: str | None) -> Resultat:
    """Le domaine ROS doit etre celui du mode : 42 = vrai robot, 43 = simulation."""
    attendu = DOMAINE[mode]
    if domaine_env != str(attendu):
        return Resultat(
            "domaine",
            False,
            f"ROS_DOMAIN_ID={domaine_env} alors que le mode {mode} exige {attendu}. "
            "Passer par `roby` ou sourcer tools/pc/roby_ros_env.sh (ROBY_SIM=1 pour la simulation).",
        )
    return Resultat("domaine", True, f"{attendu} ({mode})")


def lire_urdf(urdf: str) -> tuple[list[str], dict[str, float]]:
    """Extrait du /robot_description les plugins ros2_control et la pose initiale (= nid)."""
    racine = ET.fromstring(urdf)
    plugins: list[str] = []
    initiale: dict[str, float] = {}
    for bloc in racine.iter("ros2_control"):
        for plugin in bloc.iter("plugin"):
            if plugin.text:
                plugins.append(plugin.text.strip())
        for joint in bloc.iter("joint"):
            for etat in joint.iter("state_interface"):
                if etat.get("name") != "position":
                    continue
                for param in etat.iter("param"):
                    if param.get("name") == "initial_value" and param.text:
                        initiale[joint.get("name", "")] = float(param.text)
    return plugins, initiale


def origines_urdf(urdf: str) -> dict[str, tuple[tuple[float, ...], tuple[float, ...]]]:
    """Origine (xyz, rpy) de chaque articulation d'un URDF — de quoi comparer deux geometries."""
    racine = ET.fromstring(urdf)
    out: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {}
    for j in racine.iter("joint"):
        nom = j.get("name")
        if not nom or j.find("parent") is None:
            continue  # les <joint> de ros2_control n'ont pas de filiation : ce ne sont pas des repères
        o = j.find("origin")
        xyz = o.get("xyz", "0 0 0") if o is not None else "0 0 0"
        rpy = o.get("rpy", "0 0 0") if o is not None else "0 0 0"
        out[nom] = (tuple(float(v) for v in xyz.split()), tuple(float(v) for v in rpy.split()))
    return out


def chaine_du_depot():
    """Chaine cinematique du depot (roby_cinematique). Import tardif : numpy + URDF presente.

    Volontairement en echec franc si l'URDF manque (ADR-005) : une porte rouge, jamais un repli.
    """
    sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
    import roby_cinematique

    return roby_cinematique.CHAINE, roby_cinematique.CHEMIN_URDF


def verifier_geometrie(urdf_publie: str | None, chaine=None, chemin: str = "", tol: float = 1e-6) -> Resultat:
    """La geometrie publiee par la stack doit etre celle du depot (ADR-005, 2026-09-20).

    Sans cette porte, une machine oubliee lors d'un deploiement commande le bras avec une
    description perimee : la garde, la teleop et MoveIt calculent alors sur des bras differents,
    et rien ne le signale. C'est ce qui a coute une correction d'URDF de plus de 10 cm.
    """
    if urdf_publie is None:
        return Resultat("geometrie", False, "/robot_description illisible")
    if chaine is None:
        try:
            chaine, chemin = chaine_du_depot()
        except Exception as e:  # URDF absente, numpy absent, chaine illisible
            return Resultat("geometrie", False, f"cinematique du depot illisible : {e}")
    publie = origines_urdf(urdf_publie)
    ecarts = []
    for seg in chaine:
        nom = seg["nom"]
        if nom not in publie:
            ecarts.append(f"{nom} absente de l'URDF publiée")
            continue
        xyz, rpy = publie[nom]
        for etiquette, a, b in (("xyz", seg["xyz"], xyz), ("rpy", seg["rpy"], rpy)):
            if len(a) != len(b) or any(abs(float(x) - y) > tol for x, y in zip(a, b)):
                mm = max(abs(float(x) - y) for x, y in zip(a, b)) if len(a) == len(b) else float("nan")
                ecarts.append(f"{nom}.{etiquette} écart {mm * 1000:.1f} mm/mrad")
    if ecarts:
        return Resultat(
            "geometrie",
            False,
            "l'URDF publiée DIFFÈRE de celle du dépôt : "
            + " ; ".join(ecarts[:4])
            + (f" (+{len(ecarts) - 4})" if len(ecarts) > 4 else "")
            + ". Redéployer la machine qui publie /robot_description avant tout mouvement.",
        )
    return Resultat("geometrie", True, f"identique au dépôt ({len(chaine)} segments, {os.path.basename(chemin)})")


def verifier_materiel(mode: str, plugins: list[str], nom: str = "materiel") -> Resultat:
    """Le materiel doit etre celui du mode, et lui seul.

    Appelee deux fois : sur ce que le controller_manager a REELLEMENT charge (`materiel`), et
    sur le /robot_description publie (`urdf`) — un fantome qui y publie l'URDF mock ne gene pas
    la stack en cours mais empoisonne la prochaine relance (course au mock).
    """
    attendu, autre = PLUGIN[mode], PLUGIN["sim" if mode == "reel" else "reel"]
    if not plugins:
        return Resultat(nom, False, "aucun materiel ros2_control actif")
    if autre in plugins:
        return Resultat(
            nom,
            False,
            f"{autre} alors que le mode est {mode} "
            + ("(steppers MORTS : course au mock)" if mode == "reel" else "(VRAI robot vu depuis la simulation)"),
        )
    if any(p != attendu for p in plugins):
        return Resultat(nom, False, f"materiel inattendu : {', '.join(plugins)}")
    return Resultat(nom, True, attendu)


def verifier_controleurs(etats: dict[str, str] | None) -> Resultat:
    """arm_controller et joint_state_broadcaster doivent etre actifs."""
    if etats is None:
        return Resultat("controleurs", False, "controller_manager ne repond pas")
    requis = ["arm_controller", "joint_state_broadcaster"]
    mauvais = [f"{c}={etats.get(c, 'absent')}" for c in requis if etats.get(c) != "active"]
    if mauvais:
        return Resultat("controleurs", False, ", ".join(mauvais))
    return Resultat("controleurs", True, "arm_controller + joint_state_broadcaster actifs")


def verifier_publishers(topic: str, nombre: int) -> Resultat:
    """Un seul publisher : sinon un fantome ecrit l'etat ou l'URDF (regles 1 et 4)."""
    if nombre == 1:
        return Resultat(f"publishers {topic}", True, "1")
    if nombre == 0:
        return Resultat(f"publishers {topic}", False, "0 : la stack ne tourne pas")
    return Resultat(
        f"publishers {topic}",
        False,
        f"{nombre} : un fantome publie a cote de la stack (`ros2 topic info {topic} --verbose`)",
    )


def ecart_max_deg(q: dict[str, float], ref: dict[str, float]) -> float:
    """Plus grand ecart articulaire, en degres, sur les 5 axes."""
    return max(abs(math.degrees(q[j] - ref[j])) for j in JOINTS)


def formater_pose(q: dict[str, float]) -> str:
    """Pose en degres, format de la skill : 89.53 / 52.59 / ..."""
    return " / ".join(f"{math.degrees(q[j]):.2f}" for j in JOINTS)


def verifier_pose(q: dict[str, float], ref: dict[str, float], nom: str, tol_deg: float) -> Resultat:
    """La pose lue est-elle `ref` a `tol_deg` pres ?"""
    manquants = [j for j in JOINTS if j not in q or j not in ref]
    if manquants:
        return Resultat(nom, False, f"axes absents : {', '.join(manquants)}")
    ecart = ecart_max_deg(q, ref)
    if ecart > tol_deg:
        return Resultat(nom, False, f"{formater_pose(q)} deg, ecart {ecart:.2f} deg > {tol_deg} deg")
    return Resultat(nom, True, f"{formater_pose(q)} deg")


def objets_attendus(scene: str) -> list[str]:
    """Identifiants des obstacles d'une scene roby_environments (fichier YAML installe)."""
    import yaml  # type: ignore[import-untyped]
    from ament_index_python.packages import get_package_share_directory

    chemin = os.path.join(get_package_share_directory("roby_environments"), "environments", f"{scene}.yaml")
    with open(chemin) as f:
        return [str(o["name"]) for o in yaml.safe_load(f)["objects"]]


def verifier_scene(charges: list[str] | None, attendus: list[str], scene: str) -> Resultat:
    """Tous les obstacles de la scene doivent etre dans move_group."""
    if charges is None:
        return Resultat("scene", False, "move_group ne repond pas (/get_planning_scene)")
    manquants = sorted(set(attendus) - set(charges))
    if manquants:
        return Resultat(
            "scene",
            False,
            f"{len(attendus) - len(manquants)}/{len(attendus)} obstacles '{scene}' charges "
            f"(`roby scene` pour recharger)",
        )
    return Resultat("scene", True, f"'{scene}' {len(attendus)}/{len(attendus)} obstacles")


def verifier_cameras(hz: dict[str, float]) -> Resultat:
    """Les 2 flux doivent arriver au PC (sinon : episodes enregistres SANS images)."""
    trop_lents = {k: v for k, v in hz.items() if v < CAMERA_HZ_MIN}
    detail = ", ".join(f"{k} {v:.1f} Hz" for k, v in hz.items())
    if trop_lents:
        return Resultat("cameras", False, f"{detail} (minimum {CAMERA_HZ_MIN:.0f} Hz)")
    return Resultat("cameras", True, detail)


def _etat_bras(tourne: bool, n_joint_states: int, q: dict[str, float], nid: dict[str, float]) -> Resultat | None:
    """None si le bras est lisible ET au nid ; sinon la raison de ne pas toucher a la stack."""
    if n_joint_states != 1:
        return Resultat("bras", False, f"pose illisible : {n_joint_states} publishers sur /joint_states (fantome ?)")
    if not q or not nid:
        return Resultat("bras", False, "pose ou nid illisible alors que la stack tourne")
    hors = verifier_pose(q, nid, "bras", TOL_NID_DEG)
    if not hors.ok:
        return Resultat("bras", False, f"HORS du nid ({formater_pose(q)} deg)")
    return None


def decision_relance(tourne: bool, n_joint_states: int, q: dict[str, float], nid: dict[str, float]) -> Resultat:
    """Peut-on (re)lancer la stack REELLE ?

    Au demarrage, RobySystem declare que le bras est au nid (open-loop : aucun capteur ne le
    verifie). Relancer une stack dont le bras n'est PAS au nid fausse la reference de tous les
    axes : a-coup violent au premier mouvement. Donc : stack absente => seule la confirmation
    humaine compte ; stack presente => la pose lue doit etre celle du nid.
    """
    if not tourne:
        return Resultat("relance", True, "aucune stack en service (tete au nid : confirmation humaine)")
    raison = _etat_bras(tourne, n_joint_states, q, nid)
    if raison is not None:
        return Resultat(
            "relance",
            False,
            f"{raison.message} : relancer fausserait la reference des axes. Ramener le bras au nid "
            "(pose 'sortie' par MoveIt : `roby_moveit_seq.sh sortie`, puis `roby rentrer --go`), puis relancer.",
        )
    return Resultat("relance", True, "stack en service, bras au nid : relance sure")


def decision_arret(
    tourne: bool, n_joint_states: int, q: dict[str, float], nid: dict[str, float], hors_nid: bool
) -> Resultat:
    """Peut-on arreter la stack REELLE ? Hors du nid : seulement si on l'assume (`--hors-nid`)."""
    if not tourne:
        return Resultat("arret", True, "aucune stack en service")
    raison = _etat_bras(tourne, n_joint_states, q, nid)
    if raison is None:
        return Resultat("arret", True, "bras au nid")
    if hors_nid:
        return Resultat(
            "arret", True, f"{raison.message} — assume (--hors-nid) : remettre la tete au nid avant de relancer"
        )
    return Resultat(
        "arret",
        False,
        f"{raison.message}. Apres l'arret, la relance exigera de remettre la tete au nid a la main. "
        "Rentrer d'abord (`roby_moveit_seq.sh sortie` puis `roby rentrer --go`) ou assumer avec `--hors-nid`.",
    )


# --------------------------------------------------------------------------------------------
# Lecture ROS : ne publie rien, n'appelle que des services de lecture
# --------------------------------------------------------------------------------------------


class Sonde:
    """Lit l'etat de la stack depuis un noeud rclpy existant."""

    def __init__(self, node) -> None:
        self.node = node

    def _spin(self, duree: float, fin=None) -> None:
        import rclpy

        t0 = time.monotonic()
        while time.monotonic() - t0 < duree:
            if fin is not None and fin():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def compter_publishers(self, topic: str, duree: float = 2.5) -> int:
        """Maximum observe sur `duree` : la decouverte DDS arrive par vagues."""
        vu = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < duree:
            vu = max(vu, self.node.count_publishers(topic))
            self._spin(0.1)
        return vu

    def robot_description(self, timeout: float = 5.0) -> str | None:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        recu: dict[str, str] = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        sub = self.node.create_subscription(String, "/robot_description", lambda m: recu.update(u=m.data), qos)
        self._spin(timeout, lambda: "u" in recu)
        self.node.destroy_subscription(sub)
        return recu.get("u")

    def joint_states(self, timeout: float = 5.0) -> dict[str, float]:
        from sensor_msgs.msg import JointState

        q: dict[str, float] = {}
        sub = self.node.create_subscription(
            JointState, "/joint_states", lambda m: q.update(zip(m.name, m.position)), 10
        )
        self._spin(timeout, lambda: all(j in q for j in JOINTS))
        self.node.destroy_subscription(sub)
        return q

    def cameras_hz(self, duree: float = 4.0) -> dict[str, float]:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        compte = dict.fromkeys(CAMERAS, 0)
        subs = [
            self.node.create_subscription(
                CompressedImage, topic, lambda m, k=k: compte.__setitem__(k, compte[k] + 1), qos_profile_sensor_data
            )
            for k, topic in CAMERAS.items()
        ]
        self._spin(1.5)  # decouverte
        compte = dict.fromkeys(CAMERAS, 0)
        self._spin(duree)
        for s in subs:
            self.node.destroy_subscription(s)
        return {k: v / duree for k, v in compte.items()}

    def _appel(self, client, requete, timeout: float):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(requete)
        self._spin(timeout, fut.done)
        return fut.result() if fut.done() else None

    def materiel_charge(self, timeout: float = 5.0) -> list[str] | None:
        """Plugins que le controller_manager a REELLEMENT charges (et non ce que dit un URDF)."""
        from controller_manager_msgs.srv import ListHardwareComponents

        cli = self.node.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
        res = self._appel(cli, ListHardwareComponents.Request(), timeout)
        self.node.destroy_client(cli)
        return None if res is None else [c.plugin_name for c in res.component if c.state.label == "active"]

    def controleurs(self, timeout: float = 5.0) -> dict[str, str] | None:
        from controller_manager_msgs.srv import ListControllers

        cli = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        res = self._appel(cli, ListControllers.Request(), timeout)
        self.node.destroy_client(cli)
        return None if res is None else {c.name: c.state for c in res.controller}

    def etat_stack(self, processus_distant: bool = False) -> tuple[bool, int, dict[str, float], dict[str, float]]:
        """(tourne, publishers /joint_states, pose lue, pose du nid).

        PRUDENT : la stack « tourne » des qu'UN indice le montre (controleur, topics, processus
        vu sur le Pi5). Un rate DDS ne doit jamais faire conclure « rien ne tourne » : ce serait
        autoriser une relance sans verifier que le bras est au nid.
        """
        n_js = self.compter_publishers("/joint_states")
        n_rd = self.compter_publishers("/robot_description", duree=0.5)
        cm = self.materiel_charge(timeout=3.0)
        tourne = processus_distant or cm is not None or n_js > 0 or n_rd > 0
        q: dict[str, float] = {}
        nid: dict[str, float] = {}
        if tourne:
            urdf = self.robot_description(timeout=3.0)
            if urdf is not None:
                nid = lire_urdf(urdf)[1]
            q = self.joint_states(timeout=3.0)
        return tourne, n_js, q, nid

    def objets_scene(self, timeout: float = 5.0) -> list[str] | None:
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene

        cli = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self._appel(cli, req, timeout)
        self.node.destroy_client(cli)
        return None if res is None else [o.id for o in res.scene.world.collision_objects]

    def pose_valide(self, q: dict[str, float], timeout: float = 5.0) -> bool | None:
        from moveit_msgs.srv import GetStateValidity

        cli = self.node.create_client(GetStateValidity, "/check_state_validity")
        req = GetStateValidity.Request()
        req.group_name = "arm"
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [q[j] for j in JOINTS]
        res = self._appel(cli, req, timeout)
        self.node.destroy_client(cli)
        return None if res is None else bool(res.valid)


# --------------------------------------------------------------------------------------------
# Portes composees
# --------------------------------------------------------------------------------------------


def porte_stack(node, mode: str | None = None) -> tuple[list[Resultat], dict[str, float], dict[str, float]]:
    """Porte minimale de tout programme qui commande le bras.

    Renvoie (resultats, pose lue, pose du nid) ; les poses sont vides si illisibles.
    """
    mode = mode or mode_demande()
    sonde = Sonde(node)
    res = [verifier_domaine(mode, os.environ.get("ROS_DOMAIN_ID"))]
    charge = sonde.materiel_charge()
    if charge is None:
        res.append(Resultat("materiel", False, "controller_manager ne repond pas : la stack ne tourne pas"))
    else:
        res.append(verifier_materiel(mode, charge))
    res.append(verifier_controleurs(sonde.controleurs()))
    res.append(verifier_publishers("/robot_description", sonde.compter_publishers("/robot_description")))
    res.append(verifier_publishers("/joint_states", sonde.compter_publishers("/joint_states", duree=0.5)))
    urdf = sonde.robot_description()
    nid: dict[str, float] = {}
    if urdf is None:
        res.append(Resultat("urdf", False, "/robot_description illisible"))
    else:
        plugins, nid = lire_urdf(urdf)
        res.append(verifier_materiel(mode, plugins, nom="urdf"))
    res.append(verifier_geometrie(urdf))
    q = sonde.joint_states()
    if not all(j in q for j in JOINTS):
        res.append(Resultat("pose", False, "/joint_states muet"))
        q = {}
    return res, q, nid


def porte_scene(node, scene: str = SCENE_DEFAUT) -> Resultat:
    """Tous les obstacles de `scene` sont-ils dans move_group ?"""
    return verifier_scene(Sonde(node).objets_scene(), objets_attendus(scene), scene)


def exiger(node, qui: str, scene: bool = False, depart: dict[str, float] | None = None) -> dict[str, float]:
    """Porte d'entree des programmes de commande : quitte (code 2) si la stack n'est pas saine.

    `scene`  : exiger aussi la scene de collision (ce qui passe par MoveIt ou la garde).
    `depart` : exiger que le bras soit a cette pose (0,5 deg) — ex. premier point d'une trajectoire.
    Renvoie la pose lue.
    """
    res, q, _ = porte_stack(node)
    if scene:
        res.append(porte_scene(node))
    if depart is not None and q:
        res.append(verifier_pose(q, depart, "pose de depart", 0.5))
    echecs = [r for r in res if not r.ok]
    if echecs:
        print(f"❌ {qui} REFUSE DE DEMARRER — la stack n'est pas dans l'etat attendu :", file=sys.stderr)
        for r in echecs:
            print(f"   - {r.nom} : {r.message}", file=sys.stderr)
        print("   Diagnostic complet : `roby status`. On corrige l'etat, on ne contourne pas.", file=sys.stderr)
        raise SystemExit(2)
    return q
