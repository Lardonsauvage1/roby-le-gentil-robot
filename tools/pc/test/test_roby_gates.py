"""Logique pure des portes (sans ROS) : /usr/bin/python3 -m pytest tools/pc/test"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import roby_gates as g  # noqa: E402

NID = {"joint_1": 1.5627, "joint_2": 0.9179, "joint_3": 0.4520, "joint_4": 0.0, "joint_5": 0.2009}


def urdf(plugins, initiale=NID):
    joints = "".join(
        f'<joint name="{j}"><command_interface name="position"/>'
        f'<state_interface name="position"><param name="initial_value">{v}</param></state_interface>'
        f'<state_interface name="velocity"/></joint>'
        for j, v in initiale.items()
    )
    blocs = "".join(
        f'<ros2_control name="S{i}" type="system"><hardware><plugin>{p}</plugin></hardware>{joints}</ros2_control>'
        for i, p in enumerate(plugins)
    )
    # un <joint> URDF classique, homonyme, ne doit pas etre confondu avec ceux de ros2_control
    return f'<robot name="r"><joint name="joint_1" type="revolute"/>{blocs}</robot>'


def test_mode_et_domaine():
    assert g.mode_demande({}) == "reel"
    assert g.mode_demande({"ROBY_SIM": "1"}) == "sim"
    # sans ROBY_SIM (script lance a la main, banc de test) : le domaine decide
    assert g.mode_demande({"ROS_DOMAIN_ID": "43"}) == "sim"
    assert g.mode_demande({"ROS_DOMAIN_ID": "42"}) == "reel"
    # ROBY_SIM explicite l'emporte : la porte domaine signalera l'incoherence
    assert g.mode_demande({"ROBY_SIM": "0", "ROS_DOMAIN_ID": "43"}) == "reel"
    assert g.verifier_domaine("reel", "42").ok
    assert g.verifier_domaine("sim", "43").ok
    assert not g.verifier_domaine("sim", "42").ok  # simulation sur le domaine du vrai bras
    assert not g.verifier_domaine("reel", "43").ok
    assert not g.verifier_domaine("reel", None).ok


def test_lire_urdf_plugin_et_nid():
    plugins, nid = g.lire_urdf(urdf(["roby_hardware/RobySystem"]))
    assert plugins == ["roby_hardware/RobySystem"]
    assert nid == NID


def test_materiel_selon_le_mode():
    reel, mock = "roby_hardware/RobySystem", "mock_components/GenericSystem"
    assert g.verifier_materiel("reel", [reel]).ok
    assert g.verifier_materiel("sim", [mock]).ok
    # la course au mock : le vrai robot a charge le faux materiel
    r = g.verifier_materiel("reel", [mock])
    assert not r.ok and "MORTS" in r.message
    # la simulation voit le vrai robot : jamais
    assert not g.verifier_materiel("sim", [reel]).ok
    assert not g.verifier_materiel("reel", [reel, mock]).ok
    assert not g.verifier_materiel("reel", []).ok
    assert not g.verifier_materiel("reel", ["autre/Plugin"]).ok


def test_controleurs():
    assert g.verifier_controleurs({"arm_controller": "active", "joint_state_broadcaster": "active"}).ok
    r = g.verifier_controleurs({"arm_controller": "inactive", "joint_state_broadcaster": "active"})
    assert not r.ok and "arm_controller=inactive" in r.message
    assert not g.verifier_controleurs({"joint_state_broadcaster": "active"}).ok
    assert not g.verifier_controleurs(None).ok


def test_publishers():
    assert g.verifier_publishers("/joint_states", 1).ok
    assert not g.verifier_publishers("/joint_states", 0).ok
    r = g.verifier_publishers("/joint_states", 2)
    assert not r.ok and "fantome" in r.message


def test_pose_au_nid():
    assert g.verifier_pose(dict(NID), NID, "pose", g.TOL_NID_DEG).ok
    decale = dict(NID, joint_2=NID["joint_2"] + math.radians(0.2))
    r = g.verifier_pose(decale, NID, "pose", g.TOL_NID_DEG)
    assert not r.ok and "0.20" in r.message
    assert not g.verifier_pose({"joint_1": 0.0}, NID, "pose", 0.1).ok


def test_format_pose_comme_la_skill():
    assert g.formater_pose(NID) == "89.54 / 52.59 / 25.90 / 0.00 / 11.51"


def test_scene():
    attendus = ["a", "b", "c"]
    assert g.verifier_scene(["a", "b", "c", "extra"], attendus, "x").ok
    r = g.verifier_scene(["a"], attendus, "x")
    assert not r.ok and "1/3" in r.message
    assert not g.verifier_scene([], attendus, "x").ok
    assert not g.verifier_scene(None, attendus, "x").ok


HORS = dict(NID, joint_1=NID["joint_1"] - 2.0)


def test_relance_seulement_si_aucune_stack_ou_bras_au_nid():
    assert g.decision_relance(False, 0, {}, {}).ok
    assert g.decision_relance(True, 1, dict(NID), NID).ok
    r = g.decision_relance(True, 1, HORS, NID)
    assert not r.ok and "HORS du nid" in r.message and "rentrer" in r.message
    # un fantome sur /joint_states : la pose lue ne prouve rien
    assert not g.decision_relance(True, 2, dict(NID), NID).ok
    assert not g.decision_relance(True, 1, {}, NID).ok


def test_arret_hors_nid_seulement_assume():
    assert g.decision_arret(False, 0, {}, {}, hors_nid=False).ok
    assert g.decision_arret(True, 1, dict(NID), NID, hors_nid=False).ok
    assert not g.decision_arret(True, 1, HORS, NID, hors_nid=False).ok
    assert g.decision_arret(True, 1, HORS, NID, hors_nid=True).ok


def test_cameras():
    assert g.verifier_cameras({"left": 15.0, "right": 15.2}).ok
    assert not g.verifier_cameras({"left": 15.0, "right": 0.0}).ok
