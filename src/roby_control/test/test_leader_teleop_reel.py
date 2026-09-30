"""Tests de la teleop du VRAI bras (leader_teleop_reel, US-023 / ADR-003).

Sans materiel : un faux robot (/joint_states), un faux garde (/guard/status) et un faux
bras guide (/leader/joint_states) tournent dans le meme processus, sur un domaine DDS
ISOLE -- jamais le 42 de la vraie stack.
"""

import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# 89 : ni 42 (vraie stack) ni 43 (simulation isolee), ni 87/88 (tests du poignet BLDC,
# que colcon test peut lancer EN MEME TEMPS). ROBY_TEST_DOMAIN_ID pour isoler des series
# paralleles.
DOMAINE = int(os.environ.get("ROBY_TEST_DOMAIN_ID", "89"))
if DOMAINE in (42, 43):
    raise RuntimeError("domaine DDS %d reserve : jamais pour un test" % DOMAINE)
os.environ["ROS_DOMAIN_ID"] = str(DOMAINE)

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

from roby_control import leader_teleop_reel as reel  # noqa: E402
from roby_control.leader_mapping import charger  # noqa: E402
from roby_control.leader_teleop_cart import J, POSE_TRAVAIL, TeleopCart  # noqa: E402

Q0 = np.array(POSE_TRAVAIL, float)


# ------------------------------------------------------------------ fonctions pures
def test_conversion_j3_aller_retour():
    q = np.array([0.1, 0.8, -0.4, 0.0, 1.2])
    for s in (0.0, 0.9299):
        assert np.allclose(reel.vers_robot(reel.vers_modele(q, s, 0.5237), s, 0.5237), q)


def test_conversion_j3_ne_touche_que_joint_3():
    q = np.array([0.1, 0.8, -0.4, 0.2, 1.2])
    m = reel.vers_modele(q, 0.9299, 0.5237)
    assert np.allclose(np.delete(m, 2), np.delete(q, 2))
    assert m[2] == pytest.approx(0.5237 + 0.9299 * (-0.4 - 0.5237))


def test_horizon_sans_historique_reste_sur_l_ancre():
    hist = [(10.0, Q0)]
    pts = reel.horizon(hist, 10.5, 0.12, 3)
    assert [round(t, 6) for t, _ in pts] == [0.04, 0.08, 0.12]
    assert all(np.allclose(q, Q0) for _, q in pts)


def test_horizon_retarde_sans_extrapoler():
    # consigne lineaire q(t) = t sur joint_1
    hist = [(t, np.array([t, 0, 0, 0, 0.0])) for t in np.arange(0.0, 1.001, 0.02)]
    pts = reel.horizon(hist, 1.0, 0.12, 3)
    assert [q[0] for _, q in pts] == pytest.approx([0.92, 0.96, 1.0])
    # le point le plus lointain n'est jamais au-dela de la derniere consigne
    assert max(q[0] for _, q in reel.horizon(hist, 5.0, 0.12, 3)) == pytest.approx(1.0)


@pytest.mark.parametrize("etat,age,attendu", [
    (None, None, "garde absent"),
    ("OK moveit=ok pass=3 clamp=0 block=0", 5.0, "garde absent"),
    ("FROZEN[plancher : pince z=0.250 < table 0.252] pass=3 block=1", 0.1,
     "GELE [plancher : pince z=0.250 < table 0.252]"),
    ("OK moveit=off pass=0 clamp=0 block=0", 0.1, "SANS MoveIt"),
    ("??", 0.1, "illisible"),
    ("OK moveit=? pass=0 clamp=0 block=0", 0.1, ""),
    ("OK moveit=ok pass=9 clamp=1 block=0", 0.1, ""),
])
def test_lire_garde(etat, age, attendu):
    m = reel.lire_garde(etat, age, 1.5)
    assert (attendu in m) if attendu else m == ""


# ------------------------------------------------------------------ banc ROS
@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init(domain_id=DOMAINE)
    yield
    rclpy.shutdown()


class Banc(Node):
    """Faux robot + faux garde + faux bras guide."""

    def __init__(self):
        super().__init__("banc_teleop_reel")
        self.cal = charger()
        self.q_robot = Q0.copy()          # ce que publie le robot (espace ROBOT)
        self.q_guide = Q0.copy()          # pose Roby equivalente au guide
        self.cible_guide = None           # si posee : la main y va a `vitesse_main`
        self.vitesse_main = 1.0           # rad/s
        self.garde = "OK moveit=ok pass=0 clamp=0 block=0"
        self.guide_actif = True
        self.robot_actif = True
        self.suivre = False               # le faux robot suit-il les consignes ?
        self.recues = []                  # trajectoires recues du noeud
        self.js_etrangers = 0
        self.p_js = self.create_publisher(JointState, "/joint_states", 10)
        self.p_leader = self.create_publisher(JointState, "/leader/joint_states", 10)
        self.p_garde = self.create_publisher(String, "/guard/status", 10)
        self.create_subscription(JointTrajectory, reel.TOPIC_GARDE, self._traj, 10)
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.create_timer(0.01, self._robot)
        self.create_timer(0.02, self._leader)
        self.create_timer(0.2, self._statut)

    def _traj(self, m):
        self.recues.append((time.monotonic(), m))
        if self.suivre and m.points:
            self.q_robot = np.array(m.points[0].positions, float)

    def _js(self, m):
        if "banc" not in (m.header.frame_id or ""):
            self.js_etrangers += 1

    def _robot(self):
        if self.robot_actif:
            m = JointState()
            m.header.frame_id = "banc"
            m.name = list(J) + ["gripper"]
            m.position = [float(v) for v in self.q_robot] + [0.0]
            self.p_js.publish(m)

    def _leader(self):
        if not self.guide_actif:
            return
        if self.cible_guide is not None:
            d = self.cible_guide - self.q_guide
            pas = self.vitesse_main * 0.02
            self.q_guide = self.q_guide + np.clip(d, -pas, pas)
        m = JointState()
        par = self.cal.par_nom
        for i, nom in enumerate(J):
            j = par[nom]
            m.name.append(j.nom_leader)
            m.position.append(float(j.convertir_inverse(float(self.q_guide[i]))[0]))
        self.p_leader.publish(m)

    def _statut(self):
        if self.garde is not None:
            self.p_garde.publish(String(data=self.garde))


@pytest.fixture
def banc():
    b = Banc()
    n = reel.TeleopReel()
    ex = SingleThreadedExecutor()
    ex.add_node(b)
    ex.add_node(n)

    def tourner(duree=None, jusqua=None, max_s=5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < (duree if duree is not None else max_s):
            ex.spin_once(timeout_sec=0.005)
            if jusqua is not None and jusqua():
                return True
        return jusqua is None

    b.tourner, b.n = tourner, n
    tourner(duree=0.6)                     # decouverte DDS + premieres donnees
    yield b
    ex.shutdown()
    n.destroy_node()
    b.destroy_node()


def embrayer(b, oui=True):
    return b.n._srv_embrayage(SetBool.Request(data=oui), SetBool.Response())


def test_ne_publie_jamais_sur_joint_states(banc):
    assert banc.n.pub is None
    noms = [t for t, _ in banc.n.get_publisher_names_and_types_by_node(
        "leader_teleop_cart", "/")]
    assert "/joint_states" not in noms and reel.TOPIC_GARDE in noms
    assert embrayer(banc).success
    banc.cible_guide = Q0 + np.array([0.05, 0, 0, 0, 0])
    banc.tourner(duree=0.5)
    assert banc.js_etrangers == 0


def test_refus_sans_garde(banc):
    banc.garde = None
    banc.n.t_garde = None
    r = embrayer(banc)
    assert not r.success and "garde absent" in r.message


def test_refus_garde_gele(banc):
    banc.garde = "FROZEN[COLLISION (MoveIt)] pass=0 block=1"
    banc.tourner(duree=0.5)
    r = embrayer(banc)
    assert not r.success and "GELE" in r.message


def test_refus_garde_sans_moveit(banc):
    banc.garde = "OK moveit=off pass=0 clamp=0 block=0"
    banc.tourner(duree=0.5)
    r = embrayer(banc)
    assert not r.success and "MoveIt" in r.message


def test_refus_sans_joint_states(banc):
    banc.robot_actif = False
    banc.tourner(duree=0.5)
    r = embrayer(banc)
    assert not r.success and "joint_states" in r.message


def test_refus_si_un_autre_noeud_publie_vers_le_garde(banc):
    autre = banc.create_publisher(JointTrajectory, reel.TOPIC_GARDE, 10)
    banc.tourner(duree=0.5)
    r = embrayer(banc)
    assert not r.success and "autre" in r.message
    banc.destroy_publisher(autre)


def test_pose_de_travail_refusee(banc):
    r = banc.n._srv_pose_travail(Trigger.Request(), Trigger.Response())
    assert not r.success


def test_echelle_plafonnee(banc):
    from rclpy.parameter import Parameter
    r = banc.n.set_parameters([Parameter("echelle", value=2.0)])[0]
    assert not r.successful
    assert banc.n.set_parameters([Parameter("echelle", value=0.5)])[0].successful


def test_ancre_sur_la_position_reelle_meme_compensee(banc, monkeypatch):
    monkeypatch.setattr(reel, "J3_SCALE", 0.9299)
    banc.q_robot = Q0 + np.array([0.1, -0.05, 0.08, 0.02, -0.1])   # loin de la pose du noeud
    banc.tourner(duree=0.3)
    assert embrayer(banc).success
    banc.recues.clear()
    assert banc.tourner(jusqua=lambda: len(banc.recues) >= 5)
    for _, m in banc.recues:
        assert [round(p.time_from_start.nanosec * 1e-9, 3) for p in m.points] == \
            [0.04, 0.08]
        for p in m.points:       # guide immobile : on ne bouge pas d'un micron
            assert np.allclose(p.positions, banc.q_robot, atol=1e-6)


def test_suit_le_guide_a_vitesse_plafonnee(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.vitesse_main = 3.0                               # main rapide
    banc.cible_guide = Q0 + np.array([0.3, 0, 0, 0, 0])   # base : axe recopie, 1:1
    banc.recues.clear()
    cible = Q0[0] + 0.3
    assert banc.tourner(jusqua=lambda: banc.recues and abs(
        banc.recues[-1][1].points[-1].positions[0] - cible) < 1e-3, max_s=3.0)
    assert banc.n.embraye                                 # main rapide : pas de faux saut
    t = np.array([r[0] for r in banc.recues])
    j1 = np.array([r[1].points[-1].positions[0] for r in banc.recues])
    vit = np.diff(j1) / np.maximum(np.diff(t), 1e-3)
    # 0,4 rad/s de consigne, a la gigue de reception pres
    assert float(np.median(vit[vit > 0])) == pytest.approx(0.4, abs=0.08)
    assert float((t[-1] - t[0])) > 0.3 / 0.4 * 0.8       # ~0,75 s, pas instantane


def rejouer_guide(monkeypatch):
    """Rejoue le geste ENREGISTRE du 2026-09-13, horloge simulee, faux robot parfait.

    Retourne (debrayages, plus grand pas articulaire par cycle)."""
    import json
    from rclpy.parameter import Parameter
    d = json.load(open(os.path.join(os.path.dirname(__file__), "data",
                                    "guide_20260913_decrochage.json")))

    class Horloge:
        t = 0.0
        monotonic = staticmethod(lambda: Horloge.t)
    monkeypatch.setattr(reel, "time", Horloge)
    n = reel.TeleopReel()
    try:
        n.set_parameters([Parameter("echelle", value=float(d["echelle"]))])
        n._autres_emetteurs = lambda: 0
        n._maintenir = lambda: None
        n._envoyer = lambda t: None
        n.pub_suivi.publish = lambda m: None
        n.garde_etat = "OK moveit=ok pass=0 clamp=0 block=0"
        n.q_mes = np.array(d["q_robot"], float)
        guide, i, t = d["guide"], 0, d["guide"][0][0]
        debrayages, pas_max, engage = 0, 0.0, False
        while t < guide[-1][0]:
            Horloge.t = t
            while i < len(guide) and guide[i][0] <= t:
                n._cb(JointState(name=d["noms"], position=guide[i][1]))
                i += 1
            if n.embraye:
                n.q_mes = reel.vers_robot(n.q)
            n.t_mes = n.t_garde = t
            if not engage and t >= 0.0:
                assert embrayer(type("B", (), {"n": n})).success
                engage = True
            avant, q0 = n.embraye, np.array(n.q, float)
            n._tick()
            if avant and not n.embraye:
                debrayages += 1
            elif avant:
                pas_max = max(pas_max, float(np.abs(np.asarray(n.q) - q0).max()))
            t += n.dt
        return debrayages, pas_max
    finally:
        n.destroy_node()


def test_geste_enregistre_ne_decroche_plus(monkeypatch):
    """Cas du 2026-09-13 (echelle 1:10) : le guide incline a 40-75 deg/s. Avec une IK qui
    tenait aussi l'orientation, sa solution s'eloignait de la consigne bridee jusqu'au
    seuil de saut -- decrochage 4,5 s apres l'embrayage, sans vrai saut. Le poignet est
    maintenant RECOPIE (bride a vitesse_art_max) et l'IK ne place que son centre : la
    cause a disparu, sans plafond de rotation."""
    debrayages, pas_max = rejouer_guide(monkeypatch)
    assert debrayages == 0
    assert pas_max < 0.4 * 0.02 + 1e-6


# ------------------------------------------------------------------ centre du poignet
def test_centre_du_poignet_ne_depend_pas_de_joint_5():
    """joint_5 ne deplace pas son propre centre : c'est ce qui fonde le point commande."""
    from roby_tool_pickup import fk_poignet
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = rng.uniform(-2.0, 2.0, 5)
        q2 = q.copy()
        q2[4] = rng.uniform(-3.0, 3.0)
        assert np.allclose(fk_poignet(q2), fk_poignet(q), atol=1e-12)


def test_joint_4_ne_deplace_pas_le_centre_du_poignet():
    """L'hypothese qui fonde le point commande : le roulis ne bouge pas son propre centre.

    Histoire de ce test, qui vaut d'etre lue : l'URDF livree le 2026-09-20 placait
    l'origine de joint_5 a 99 mm au-dessus de l'axe de l'avant-bras, ce qui rendait
    l'hypothese FAUSSE (joint_4 emportait alors le centre de ~10 cm/rad). Trois
    verifications independantes ont montre que ce decalage n'existait pas dans la piece
    reelle, NM a confirme que l'avant-bras est droit, et l'hypothese est retablie.
    Si ce test se remet a echouer, c'est que la geometrie a re-derive : le point commande
    de la teleoperation cartesienne ne tient plus.
    """
    from roby_tool_pickup import fk_poignet
    rng = np.random.default_rng(3)
    for _ in range(200):
        q = rng.uniform(-2.0, 2.0, 5)
        q4 = q.copy()
        q4[3] = rng.uniform(-3.0, 3.0)
        assert np.allclose(fk_poignet(q4), fk_poignet(q), atol=1e-12)


def test_le_decalage_de_la_pince_est_rigide():
    """link_gripper est fixe par rapport a link_5 : la distance au centre du poignet est constante."""
    from roby_tool_pickup import fk_poignet, fkT
    rng = np.random.default_rng(1)
    d0 = None
    for _ in range(200):
        q = rng.uniform(-2.0, 2.0, 5)
        d = float(np.linalg.norm(fkT(q)[:3, 3] - fk_poignet(q)))
        d0 = d if d0 is None else d0
        assert abs(d - d0) < 1e-12


def test_axes_du_poignet_toujours_recopies(banc):
    from rclpy.parameter import Parameter
    assert banc.n._recopies() == [0, 3, 4]
    assert banc.n._actifs() == [1, 2]
    # meme si on les retire des axes recopies : ils ne deplacent pas le point commande
    banc.n.set_parameters([Parameter("axes_directs", value=[1])])
    assert banc.n._recopies() == [0, 3, 4]
    # base rendue a l'IK : elle rejoint joint_2 et joint_3
    banc.n.set_parameters([Parameter("base_directe", value=False)])
    assert banc.n._recopies() == [3, 4] and banc.n._actifs() == [0, 1, 2]


def _poignet(m):
    from roby_tool_pickup import fk_poignet
    return fk_poignet(reel.vers_modele(m.points[-1].positions))


def test_rouler_le_poignet_du_guide_ne_deplace_pas_le_centre(banc):
    """La main roule son poignet : le centre du poignet du robot ne bouge pas.

    C'est ce qui rend le point commande utilisable — les deux axes du poignet sont
    recopies du guide et n'ont aucun effet sur la position visee.
    """
    banc.suivre = True
    assert embrayer(banc).success
    p0 = _poignet(type("M", (), {"points": [type("P", (), {"positions": Q0})]}))
    banc.vitesse_main = 0.3
    banc.cible_guide = Q0 + np.array([0, 0, 0, 0.25, -0.30])  # la main tourne le poignet
    banc.recues.clear()
    assert banc.tourner(jusqua=lambda: banc.recues and np.allclose(
        banc.recues[-1][1].points[-1].positions[3:], banc.cible_guide[3:], atol=1e-3),
        max_s=4.0)
    assert banc.n.embraye
    for _, m in banc.recues:
        assert np.linalg.norm(_poignet(m) - p0) < 1e-4      # le centre ne bouge pas


def test_le_roulis_du_guide_ne_fait_pas_deriver_les_axes_1_a_3(banc):
    """Le roulis passe par les axes recopies : l'IK n'a pas a remuer l'epaule ni le coude."""
    banc.suivre = True
    assert embrayer(banc).success
    banc.vitesse_main = 0.3
    banc.cible_guide = Q0 + np.array([0, 0, 0, 0.25, -0.30])
    banc.recues.clear()
    banc.tourner(duree=2.0)
    for _, m in banc.recues:
        assert np.allclose(m.points[-1].positions[:3], Q0[:3], atol=0.02)


def test_ik_place_le_centre_du_poignet(banc):
    """La main deplace son poignet : celui du robot suit, du meme deplacement a 1:1."""
    from roby_tool_pickup import fk_poignet
    banc.suivre = True
    assert embrayer(banc).success
    banc.vitesse_main = 0.3
    banc.cible_guide = Q0 + np.array([0, -0.10, 0.10, 0, 0])   # pince 6,8 cm au-dessus
    voulu = fk_poignet(banc.cible_guide)                    # k = 1, robot parti de Q0
    banc.recues.clear()
    banc.tourner(duree=4.0)
    assert banc.n.embraye
    assert np.linalg.norm(_poignet(banc.recues[-1][1]) - voulu) < 2e-3


def test_poignet_a_1_1_meme_a_echelle_reduite(banc):
    from rclpy.parameter import Parameter
    banc.n.set_parameters([Parameter("echelle", value=0.5)])
    banc.suivre = True
    assert embrayer(banc).success
    banc.vitesse_main = 0.3
    banc.cible_guide = Q0 + np.array([0.20, 0, 0, 0, 0.20])
    banc.recues.clear()
    banc.tourner(duree=3.0)
    q = np.asarray(banc.recues[-1][1].points[-1].positions)
    assert q[0] - Q0[0] == pytest.approx(0.10, abs=2e-3)    # base : a l'echelle
    assert q[4] - Q0[4] == pytest.approx(0.20, abs=2e-3)    # poignet : 1:1


def _direction_descente(q):
    """Pas du guide (joint_2, 3, 5) qui fait descendre la pince sans la tourner."""
    from roby_tool_pickup import fkT, rotvec
    T0, eps, cols = fkT(q), 1e-5, []
    for i in (1, 2, 4):
        T = fkT(q + eps * np.eye(5)[i])
        cols.append(np.concatenate([[(T[2, 3] - T0[2, 3]) / eps],
                                    rotvec(T0[:3, :3].T @ T[:3, :3]) / eps]))
    x = np.linalg.lstsq(np.array(cols).T, np.array([-1.0, 0, 0, 0]), rcond=None)[0]
    d = np.zeros(5)
    d[[1, 2, 4]] = x / np.linalg.norm(x)
    return d


def test_plancher_virtuel_le_bras_glisse_sans_faire_geler_le_garde(banc):
    from roby_tool_pickup import fkT
    banc.suivre = True
    assert embrayer(banc).success
    banc.vitesse_main = 0.3
    d = _direction_descente(Q0)
    banc.cible_guide = Q0 + 0.8 * d                       # la main s'enfonce loin
    banc.recues.clear()
    banc.tourner(duree=4.0)
    assert banc.n.embraye
    z = [fkT(reel.vers_modele(m.points[-1].positions))[2, 3] for _, m in banc.recues]
    p = fkT(reel.vers_modele(banc.recues[-1][1].points[-1].positions))[:3, 3]
    plancher_garde = reel.O._z_pick(p[0], p[1]) - 0.03
    assert min(z) > plancher_garde + 0.02                 # jamais sous la marge (2,5 cm)
    assert min(z) < plancher_garde + 0.03                 # mais il est bien descendu
    # la main remonte de 2 cm : le bras decolle tout de suite (pas de zone morte)
    z_bas = z[-1]
    banc.cible_guide = banc.q_guide - 0.05 * d
    banc.recues.clear()
    banc.tourner(duree=1.0)
    z2 = fkT(reel.vers_modele(banc.recues[-1][1].points[-1].positions))[2, 3]
    assert z2 > z_bas + 0.01


def test_plancher_virtuel_ne_souleve_pas_le_bras(banc):
    """Bras deja sous le plancher de la teleop : il ne doit pas remonter seul."""
    banc.n.marge_plancher = 0.5                           # plancher bien au-dessus
    banc.suivre = True
    assert embrayer(banc).success
    banc.recues.clear()
    banc.tourner(duree=1.0)
    for _, m in banc.recues:
        assert np.allclose(m.points[-1].positions, Q0, atol=1e-6)


def test_debraye_si_le_guide_saute(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.q_guide = Q0 + np.array([0.4, 0, 0, 0, 0])      # 0,4 rad en 20 ms : glitch
    assert banc.tourner(jusqua=lambda: not banc.n.embraye, max_s=1.0)


def test_debraye_si_le_guide_se_tait(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.guide_actif = False
    assert banc.tourner(jusqua=lambda: not banc.n.embraye, max_s=2.0)
    banc.tourner(duree=0.15)
    # dernier message : un maintien a la position mesuree, un seul point
    m = banc.recues[-1][1]
    assert len(m.points) == 1 and np.allclose(m.points[0].positions, banc.q_robot)


def test_debraye_si_le_garde_gele(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.garde = "FROZEN[COLLISION (MoveIt)] pass=5 block=1"
    assert banc.tourner(jusqua=lambda: not banc.n.embraye, max_s=1.0)


def test_debraye_si_le_garde_disparait(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.garde = None
    assert banc.tourner(jusqua=lambda: not banc.n.embraye, max_s=3.0)


def test_debraye_si_le_bras_ne_suit_pas(banc):
    banc.suivre = False                                   # bras bloque
    assert embrayer(banc).success
    banc.cible_guide = Q0 + np.array([0.4, 0, 0, 0, 0])
    t0 = time.monotonic()
    assert banc.tourner(jusqua=lambda: not banc.n.embraye, max_s=4.0)
    assert time.monotonic() - t0 > 0.5                    # pas au premier ecart


def test_ne_reprend_pas_seul(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.guide_actif = False
    assert banc.tourner(jusqua=lambda: not banc.n.embraye, max_s=2.0)
    banc.guide_actif = True
    banc.tourner(duree=0.5)
    assert not banc.n.embraye


def test_debrayer_arrete_le_bras_la_ou_il_est(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.tourner(duree=0.2)
    assert embrayer(banc, False).success
    banc.tourner(duree=0.15)
    m = banc.recues[-1][1]
    assert len(m.points) == 1 and np.allclose(m.points[0].positions, banc.q_robot)
    n = len(banc.recues)
    banc.cible_guide = Q0 + np.array([0.1, 0, 0, 0, 0])
    banc.tourner(duree=0.4)
    assert len(banc.recues) == n                          # debraye : plus rien


# ------------------------------------------------------------------ mode simulation
def _message_guide(cal, q):
    m = JointState()
    for i, nom in enumerate(J):
        j = cal.par_nom[nom]
        m.name.append(j.nom_leader)
        m.position.append(float(j.convertir_inverse(float(q[i]))[0]))
    return m


def _cycles(n, msg, cycles=10):
    for _ in range(cycles):
        n._cb(msg)
        n._tick()


def test_calibration_rechargee_embraye_ne_bouge_pas(tmp_path):
    """Revue du 2026-09-13 : recharger la calibration pendant l'embrayage reancrait sur
    une pose du guide calculee avec l'ANCIENNE calibration ; au message suivant, le bras
    bougeait (13 cm de TCP pour +10 deg sur le zero de la base, guide immobile)."""
    import yaml
    n = TeleopCart()
    try:
        cal = charger()
        msg = _message_guide(cal, Q0)
        n.q = Q0.copy()
        _cycles(n, msg, 3)
        assert n._srv_embrayage(SetBool.Request(data=True), SetBool.Response()).success
        _cycles(n, msg)
        p0 = n._p_pince(n.q).copy()
        # meme fichier, zero_urdf de la base decale de 10 deg
        src = n._fichier_calib()
        data = yaml.safe_load(open(src, encoding="utf-8"))
        for e in data["joints"]:
            if e["nom_urdf"] == "joint_1":
                e["zero_urdf"] = float(e.get("zero_urdf", 0.0)) + math.radians(10)
        tmp = tmp_path / "leader_calibration.yaml"
        tmp.write_text(yaml.safe_dump(data), encoding="utf-8")
        n._chemin, n._mtime = str(tmp), None
        n._recharger_si_change()
        assert n.cal.par_nom["joint_1"].zero_urdf != cal.par_nom["joint_1"].zero_urdf
        _cycles(n, msg, 20)                           # guide IMMOBILE
        assert n.embraye
        assert float(np.linalg.norm(n._p_pince(n.q) - p0)) < 1e-3
    finally:
        n.destroy_node()


def test_base_directe_sans_la_base_recopiee_ne_bouge_pas():
    """Revue du 2026-09-13 : base_directe avec joint_1 hors des axes recopies mettait
    l'ancre dans le plan du bras et la cible dans le monde : 6 cm des l'embrayage."""
    n = TeleopCart()
    try:
        from rclpy.parameter import Parameter
        assert n.set_parameters([Parameter("axes_directs", value=[5])])[0].successful
        # le poignet (joint_4, joint_5) est toujours recopie ; la base, elle, ne l'est pas
        assert n.base_directe and n.directs == [3, 4] and n._actifs() == [0, 1, 2]
        msg = _message_guide(charger(), Q0)
        n.q = Q0.copy()
        _cycles(n, msg, 3)
        assert n._srv_embrayage(SetBool.Request(data=True), SetBool.Response()).success
        p0 = n._p_pince(n.q).copy()
        _cycles(n, msg, 30)
        assert float(np.linalg.norm(n._p_pince(n.q) - p0)) < 1e-3
    finally:
        n.destroy_node()


def test_vrai_bras_refuse_de_changer_les_axes_recopies_embraye(banc):
    from rclpy.parameter import Parameter
    assert embrayer(banc).success
    r = banc.n.set_parameters([Parameter("axes_directs", value=[1, 5])])[0]
    assert not r.successful and "debrayer" in r.reason


def test_vrai_bras_debraye_si_la_calibration_change(banc):
    banc.suivre = True
    assert embrayer(banc).success
    banc.n._mtime = -1.0                              # « fichier modifie »
    banc.n._recharger_si_change()
    assert not banc.n.embraye


def test_bras_simule_muet_a_cote_d_un_vrai_robot():
    """BUG-008 : le noeud du bras SIMULE ne publie pas /joint_states si un autre
    publisher existe (joint_state_broadcaster du vrai robot)."""
    n = TeleopCart()
    autre = n.create_publisher(JointState, "/joint_states", 10)   # le « robot »
    recus = []
    ecoute = n.create_subscription(JointState, "/joint_states", recus.append, 10)
    ex = SingleThreadedExecutor()
    ex.add_node(n)

    def tourner(duree):
        t0 = time.monotonic()
        while time.monotonic() - t0 < duree:
            n._publier()                     # ce que fait chaque tick du noeud
            ex.spin_once(timeout_sec=0.02)

    try:
        tourner(3.0)                         # > attente de decouverte (2 s)
        assert recus == []
        n.destroy_publisher(autre)
        tourner(1.5)
        assert recus, "seul a nouveau : la simulation doit publier"
    finally:
        n.destroy_subscription(ecoute)
        ex.shutdown()
        n.destroy_node()


def test_repere_bleu_au_point_commande():
    """La boule bleue et la croix designent le meme point : le point COMMANDE, centre du
    poignet depuis le 2026-09-15. Avant le 2026-09-13, la boule etait a un autre point
    que la croix, et la croix restait rouge meme avec un suivi parfait."""
    n = TeleopCart()
    try:
        n.cible_brute = n._p_poignet(n.q)
        publies = []
        n.pub_marq.publish = publies.append
        n._marqueurs()
        spheres = {m.id: m for m in publies[0].markers if m.id in (0, 1)}
        p = spheres[1].pose.position
        assert np.allclose([p.x, p.y, p.z], n._p_poignet(n.q))
        assert spheres[0].color.g > 0.5                 # vert : suivi parfait
    finally:
        n.destroy_node()


def test_mode_simulation_inchange():
    """Le noeud du bras SIMULE publie toujours sur /joint_states, et rien vers le garde."""
    n = TeleopCart()
    try:
        assert n.pub is not None
        noms = [t for t, _ in n.get_publisher_names_and_types_by_node(
            "leader_teleop_cart", "/")]
        assert "/joint_states" in noms and reel.TOPIC_GARDE not in noms
        assert math.isclose(n.vmax, 0.25)
    finally:
        n.destroy_node()
