"""Tests de la correspondance leader -> Roby (US-019) — sans ROS ni materiel."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from roby_control.leader_mapping import JointMapping, LeaderCalibration, charger  # noqa: E402

CALIB = os.path.join(os.path.dirname(__file__), "..", "config", "leader_calibration.yaml")


def _m(**kw):
    base = dict(id=1, nom_leader="axe", nom_urdf="joint_1",
                urdf_min=-math.pi, urdf_max=math.pi)
    base.update(kw)
    return JointMapping(**base)


# ------------------------------------------------------------------ formule de base
def test_le_zero_donne_zero():
    assert _m(zero=1.234).convertir(1.234)[0] == pytest.approx(0.0)


def test_signe_inverse_le_sens():
    pos = _m(zero=0.0, signe=1.0).convertir(0.5)[0]
    neg = _m(zero=0.0, signe=-1.0).convertir(0.5)[0]
    assert pos == pytest.approx(-neg)


def test_echelle_multiplie_l_amplitude():
    assert _m(echelle=2.0).convertir(0.5)[0] == pytest.approx(1.0)
    assert _m(echelle=0.5).convertir(0.5)[0] == pytest.approx(0.25)


# ------------------------------------------------------------------ deroulement
def test_deroulement_juste_apres_le_bouclage():
    """Le point cle : 4 axes sur 6 franchissent le zero du codeur.

    Zero a 6,0 rad, position lue a 0,1 rad (juste apres le bouclage) : l'ecart REEL est
    de +0,383 rad. Une soustraction brute donnerait -5,9 rad, soit presque un tour dans
    le mauvais sens — le bras partirait en butee.
    """
    m = _m(zero=6.0)
    assert m.ecart_deroule(0.1) == pytest.approx(0.1 - 6.0 + 2 * math.pi)
    assert m.ecart_deroule(0.1) == pytest.approx(0.3832, abs=1e-3)
    assert m.convertir(0.1)[0] == pytest.approx(0.3832, abs=1e-3)


def test_deroulement_symetrique_avant_le_bouclage():
    m = _m(zero=0.1)
    assert m.ecart_deroule(6.0) == pytest.approx(-0.3832, abs=1e-3)


def test_deroulement_reste_dans_un_demi_tour():
    m = _m(zero=2.0)
    for q in [i * 0.13 for i in range(60)]:
        assert -math.pi <= m.ecart_deroule(q) <= math.pi


def test_cas_reel_pince():
    """Pince mesuree : fermee 3658 pas, ouverte 746 pas, course reelle 104,1 deg."""
    s2r = lambda st: st * 2 * math.pi / 4096
    m = _m(zero=s2r(3658), urdf_min=0.0, urdf_max=1.0,
           echelle=1.0 / 1.816233)  # normalise la course sur [0, 1]
    assert m.convertir(s2r(3658))[0] == pytest.approx(0.0, abs=1e-6)   # fermee
    assert m.convertir(s2r(746))[0] == pytest.approx(1.0, abs=1e-3)    # ouverte
    assert m.convertir(s2r(83))[0] == pytest.approx(0.44, abs=0.02)    # mi-course


# ------------------------------------------------------------------ clamp
def test_clamp_signale_le_depassement():
    m = _m(urdf_min=-1.6, urdf_max=1.6, echelle=1.0)
    q, clampe = m.convertir(3.0)
    assert q == 1.6 and clampe is True
    q, clampe = m.convertir(-3.0)
    assert q == -1.6 and clampe is True
    q, clampe = m.convertir(0.5)
    assert q == pytest.approx(0.5) and clampe is False


def test_axe_5_le_leader_deborde_la_butee_de_roby():
    """Cas reel : course leader 207,5 deg > butee Roby 183,3 deg.

    Sans clamp, un geste normal sur le bras guide commanderait hors butee.
    """
    m = _m(nom_urdf="joint_5", urdf_min=-1.6, urdf_max=1.6,
           zero=0.0, echelle=1.0, course_leader=3.621729)
    c = m.couverture()
    assert c["debordement_rad"] > 0
    assert c["inatteignable_rad"] == 0
    assert m.convertir(3.621729 / 2)[1] is True  # demi-course -> deja hors butee


def test_axe_1_une_partie_de_roby_reste_inatteignable():
    m = _m(urdf_min=-3.14159, urdf_max=3.14159, echelle=1.0, course_leader=4.121806)
    c = m.couverture()
    assert c["inatteignable_rad"] == pytest.approx(2.2 * 2 / 2, abs=0.3)
    assert c["debordement_rad"] == 0


# ------------------------------------------------------------------ fichier
def test_le_fichier_de_calibration_se_charge():
    cal = charger(CALIB)
    assert [j.nom_urdf for j in cal.joints] == [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"
    ]
    assert cal.par_nom["joint_4"].continu is True
    assert cal.par_nom["joint_5"].urdf_max == 1.6


def test_le_drapeau_non_calibre_est_signale():
    """Garde-fou : piloter avec des parametres non releves n'a aucun sens.

    On teste le MECANISME, pas l'etat du fichier a un instant donne : celui-ci evolue
    au fil de la calibration.
    """
    cal = LeaderCalibration(joints=[
        _m(nom_urdf="joint_1", calibre=True),
        _m(nom_urdf="joint_2", calibre=False),
    ])
    assert cal.non_calibres() == ["joint_2"]


def test_le_fichier_livre_est_entierement_calibre():
    """Les 6 axes ont leurs parametres releves (2026-09-05)."""
    assert charger(CALIB).non_calibres() == []


def test_signes_retenus():
    """Verrouille les signes VERIFIES EN TELEOPERATION le 2026-09-05.

    Historique utile, car il dit ce qui fait autorite : les signes avaient d'abord ete
    DEDUITS (convention de la trajectoire de sortie du nid pour l'epaule et le coude,
    cinematique directe pour les axes 4 et 5). L'essai en simulation a montre que les
    SIX partaient a l'envers : ce n'etait donc pas une erreur par axe mais une
    convention de sens globalement inversee.

    => Sur cette question, l'ESSAI prime sur la deduction. Ne pas "recorriger" ces
    valeurs a partir d'un raisonnement geometrique sans les avoir reverifiees sur le
    bras.

    2026-09-09 : joint_4 inverse a nouveau, constate avec Sam en teleoperation simulee
    (df8c071). Valeurs = table « Calibration en vigueur » de spec-teleoperation-bras-guide
    (le fichier fait foi). Ce test figeait encore joint_4 a +1 : il echouait depuis.
    """
    cal = charger(CALIB)
    attendus = {"joint_1": -1.0,   # inverse une seconde fois apres essai (2026-09-05)
                "joint_2": +1.0, "joint_3": +1.0,
                "joint_4": -1.0,   # inverse en teleoperation simulee (2026-09-09)
                "joint_5": +1.0, "gripper": -1.0}
    for nom, signe in attendus.items():
        assert cal.par_nom[nom].signe == signe, nom


def test_pose_de_reference_donne_le_centre_de_course():
    """Critere US-019 : leader en pose de reference -> angles converti = zero_urdf."""
    cal = charger(CALIB)
    angles = {j.nom_leader: j.zero for j in cal.joints}
    out, clampes = cal.convertir_tous(angles)
    assert clampes == []
    for j in cal.joints:
        assert out[j.nom_urdf] == pytest.approx(j.zero_urdf, abs=1e-6)


def test_rechargement_donne_les_memes_conversions():
    """Critere US-019 : recharger le fichier reproduit les memes angles."""
    a, b = charger(CALIB), charger(CALIB)
    angles = {j.nom_leader: 1.0 for j in a.joints}
    assert a.convertir_tous(angles)[0] == b.convertir_tous(angles)[0]


def test_conversion_de_tous_signale_les_clampes():
    cal = LeaderCalibration(joints=[
        _m(nom_leader="a", nom_urdf="joint_1", urdf_min=-1.0, urdf_max=1.0),
        _m(nom_leader="b", nom_urdf="joint_2", urdf_min=-1.0, urdf_max=1.0),
    ])
    # 1.5 rad deborde franchement la butee a 1.0.
    # (Attention : 5.0 rad NE deborderait pas par le haut — deroule autour du zero il
    #  vaut -1.28 rad, donc il serait clampe a -1.0. Le deroulement se fait oublier vite.)
    out, clampes = cal.convertir_tous({"a": 0.5, "b": 1.5})
    assert out["joint_1"] == pytest.approx(0.5)
    assert out["joint_2"] == 1.0
    assert clampes == ["joint_2"]

    out, clampes = cal.convertir_tous({"a": 0.5, "b": 5.0})
    assert out["joint_2"] == -1.0  # 5.0 rad -> -1.28 rad une fois deroule
    assert clampes == ["joint_2"]


# ------------------------------------------------------------------ echelles retenues
def test_echelles_du_fichier_couvrent_ce_qui_est_attendu():
    """Verrouille le choix du 2026-09-05 : proportionnel partout, sauf l'axe 1 en 1:1.

    Ce test existe pour qu'une modification future du YAML ne reintroduise pas
    silencieusement un debordement de butee ou une amplification du geste.
    """
    cal = charger(CALIB)

    # Axe 1 : 1:1, et CONTINU depuis le 2026-09-09 (fa2fe67) : la base couvre un tour
    # complet cote Roby, il n'y a plus de butee a deborder. (Ce test verifiait encore le
    # cas non continu d'avant : KeyError depuis.)
    a1 = cal.par_nom["joint_1"]
    assert a1.echelle == 1.0
    assert a1.fait_le_tour()

    # Ce qui compte pour la SECURITE : aucun axe ne peut commander hors butee.
    # La couverture, elle, n'est plus un critere : depuis le passage en mode JOYSTICK
    # (2026-09-05) le guide commande une VITESSE, donc tout l'espace de Roby reste
    # accessible meme si la course du guide n'en couvre qu'une partie. Les zeros et les
    # courses de reference sont d'ailleurs regles au ressenti par l'operateur, pas pour
    # maximiser une couverture.
    for nom in ("joint_2", "joint_3", "joint_5"):
        c = cal.par_nom[nom].couverture()
        assert c["debordement_rad"] == 0.0, nom

    # Axe 5 : c'etait LE cas dangereux (207,5 deg leader > 183,3 deg butee Roby).
    # Avec l'echelle retenue, la butee n'est atteinte qu'en fin de course, sans clamp.
    a5 = cal.par_nom["joint_5"]
    assert a5.echelle < 1.0
    # Le leader en butee doit rester DANS la butee de Roby, avec la marge de 1 % : sans
    # elle, la butee du leader tombait pile sur celle de Roby et un arrondi suffisait a
    # declencher le clamp a chaque passage en bout de course.
    # 2026-09-09 : zero_urdf passe a +60 deg (neutre du guide = poignet releve, regle en
    # teleoperation simulee). La correspondance n'est plus centree : cote +, le guide
    # atteint la butee de Roby AVANT la sienne, et la conversion PLAFONNE a la butee.
    # Ce test exigeait « aucun clamp » (etat du 2026-09-05) et echouait depuis. Ce qui
    # compte pour la securite reste verifie : jamais au-dela de la butee de Roby.
    demi = a5.course_leader / 2.0
    for bout in (a5.zero + demi, a5.zero - demi):
        q, _clampe = a5.convertir(bout)
        assert a5.urdf_min <= q <= a5.urdf_max, "jamais au-dela de la butee de Roby"


def test_pince_normalisee_entre_zero_et_un():
    """La pince ne sort pas un angle URDF mais une ouverture 0..1."""
    g = cal_g = charger(CALIB).par_nom["gripper"]
    assert (g.urdf_min, g.urdf_max) == (0.0, 1.0)
    assert g.echelle * g.course_leader == pytest.approx(1.0, abs=1e-3)
    assert cal_g.convertir(g.zero)[0] == pytest.approx(0.0)


# ------------------------------------------------------------------ sens inverse (US-022)
def test_aller_retour_conversion():
    """Roby -> leader -> Roby doit redonner l'angle de depart."""
    m = _m(zero=2.0, signe=-1.0, echelle=1.3, course_leader=3.0,
           urdf_min=-2.0, urdf_max=2.0)
    for q_roby in (-1.5, -0.4, 0.0, 0.7, 1.5):
        q_leader, atteignable = m.convertir_inverse(q_roby)
        assert atteignable
        assert m.convertir(q_leader)[0] == pytest.approx(q_roby)


def test_realignement_impossible_hors_course_du_leader():
    """Cas reel de l'axe 5 : le guide n'en couvre qu'une partie (~1,4 rad sur 3,2).

    Le realignement doit ECHOUER proprement, jamais forcer contre la butee du leader.
    (Ce test portait sur l'axe 1, devenu continu le 2026-09-09 : tout y est atteignable.)
    """
    cal = charger(CALIB)
    a5 = cal.par_nom["joint_5"]
    assert a5.couverture()["inatteignable_rad"] > 1.0
    lo, hi = a5.bornes_ecart()
    q_dedans = a5.zero_urdf + a5.signe * a5.echelle * (0.5 * hi)
    assert a5.convertir_inverse(q_dedans)[1] is True
    # zero_urdf = +60 deg : c'est la butee NEGATIVE de Roby qui est hors de portee.
    _, ok = a5.convertir_inverse(a5.urdf_min)
    assert ok is False


def test_realignement_possible_dans_la_plage_reellement_couverte():
    """Le realignement doit aboutir partout ou le guide peut physiquement aller.

    Depuis que les zeros sont pris sur le NEUTRE choisi par l'operateur (2026-09-05) et
    non plus au milieu geometrique de la course, les debattements ne sont plus
    symetriques : l'epaule fait -882/+1451 pas. Une partie de la course de Roby est donc
    hors d'atteinte d'un cote — ce que `convertir_inverse` signale correctement.

    Ce n'est plus genant : en mode JOYSTICK, le guide commande une VITESSE, donc tout
    l'espace de Roby reste accessible quelle que soit sa course. Ce test verifie
    seulement la coherence entre les bornes declarees et ce que la conversion accepte.
    """
    cal = charger(CALIB)
    for nom in ("joint_2", "joint_3", "joint_5"):
        j = cal.par_nom[nom]
        lo, hi = j.bornes_ecart()
        # au neutre, et a 90 % de chaque bout : toujours atteignable
        for ecart in (0.0, 0.9 * lo, 0.9 * hi):
            q = j.zero_urdf + j.signe * j.echelle * ecart
            assert j.convertir_inverse(q)[1] is True, (nom, ecart)
        # nettement au-dela d'un bout : signale comme hors d'atteinte, jamais force
        q = j.zero_urdf + j.signe * j.echelle * (hi + 0.5)
        assert j.convertir_inverse(q)[1] is False, nom
