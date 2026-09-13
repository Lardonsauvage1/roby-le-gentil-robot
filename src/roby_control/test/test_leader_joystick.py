"""Tests du mode joystick (sans ROS ni materiel)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from roby_control.leader_joystick import commande, commande_axe  # noqa: E402
from roby_control.leader_mapping import JointMapping  # noqa: E402

DEMI, MORTE, CMAX = 1.0, 0.1, 100


def test_au_zero_rien_ne_bouge():
    assert commande(0.0, DEMI, MORTE, CMAX) == (0.0, 0)


def test_zone_morte_ne_produit_ni_vitesse_ni_couple():
    """Un couple residuel dans la zone morte ferait osciller le bras au neutre."""
    for e in (-0.1, -0.05, 0.0, 0.05, 0.1):
        v, c = commande(e, DEMI, MORTE, CMAX)
        assert v == 0.0 and c == 0, e


def test_juste_au_dela_de_la_zone_morte_c_est_doux():
    v, c = commande(MORTE + 1e-6, DEMI, MORTE, CMAX)
    assert 0 < v < 0.01 and c == 0        # demarrage progressif, pas un saut


def test_a_fond_en_bout_de_course():
    v, c = commande(DEMI, DEMI, MORTE, CMAX)
    assert v == pytest.approx(1.0) and c == CMAX


def test_symetrie_des_deux_cotes():
    vp, cp = commande(+0.6, DEMI, MORTE, CMAX)
    vn, cn = commande(-0.6, DEMI, MORTE, CMAX)
    assert vp == pytest.approx(-vn) and cp == cn


def test_la_vitesse_croit_avec_l_ecart():
    vals = [commande(e, DEMI, MORTE, CMAX)[0] for e in (0.2, 0.4, 0.6, 0.8, 1.0)]
    assert vals == sorted(vals)
    assert all(0 < v <= 1.0 for v in vals)


def test_jamais_au_dela_des_bornes():
    """Meme hors course (le guide peut depasser la demi-course mesuree)."""
    for e in (5.0, -5.0, 100.0):
        v, c = commande(e, DEMI, MORTE, CMAX)
        assert -1.0 <= v <= 1.0
        assert 0 <= c <= CMAX


def test_couple_nul_pres_du_zero_maximal_au_loin():
    """L'exigence de Sam : couple ~0 pres du zero, 10 % au plus loin."""
    assert commande(0.15, DEMI, MORTE, CMAX)[1] < 10
    assert commande(0.95, DEMI, MORTE, CMAX)[1] > 90


def test_le_signe_de_la_calibration_est_applique():
    """'Guide a gauche' doit faire tourner Roby a gauche, pas l'inverse."""
    base = dict(id=1, nom_leader="a", nom_urdf="joint_1",
                urdf_min=-math.pi, urdf_max=math.pi, zero=0.0, course_leader=2.0)
    positif = JointMapping(signe=+1.0, **base)
    negatif = JointMapping(signe=-1.0, **base)
    v_pos = commande_axe(positif, 0.5, MORTE, CMAX)[0]
    v_neg = commande_axe(negatif, 0.5, MORTE, CMAX)[0]
    assert v_pos > 0 and v_neg < 0
    assert v_pos == pytest.approx(-v_neg)


# ------------------------------------------------------------------ chemin le plus court
from roby_control.leader_joystick import cible_de_rappel, pas_court  # noqa: E402


def test_pas_court_franchit_le_bouclage():
    """Cas reels du 2026-09-05, ou le servo partait a l'envers."""
    assert pas_court(3803, 883) == 1176      # epaule : +1176 et non -2920
    assert pas_court(976, 3881) == -1191     # rot. poignet : -1191 et non +2905
    assert pas_court(226, 252) == 26         # base : deja au plus court
    assert pas_court(1506, 1313) == -193     # coude : idem


def test_pas_court_est_toujours_dans_un_demi_tour():
    for a in range(0, 4096, 97):
        for z in range(0, 4096, 131):
            assert -2048 <= pas_court(a, z) <= 2048


def test_la_cible_ne_saute_jamais_loin():
    """La cible reste a portee : le servo ne peut pas tenter un grand deplacement."""
    for actuel, zero in ((3803, 883), (976, 3881), (100, 4000), (2000, 10)):
        c = cible_de_rappel(actuel, zero, pas_max=200, zone_morte_pas=50)
        assert abs(pas_court(actuel, c)) <= 200
        # ... et toujours DANS LA BONNE DIRECTION
        assert pas_court(actuel, c) * pas_court(actuel, zero) > 0


def test_dans_la_zone_morte_la_cible_est_la_position_actuelle():
    assert cible_de_rappel(1000, 1030, pas_max=200, zone_morte_pas=50) == 1000
    assert cible_de_rappel(1000, 970, pas_max=200, zone_morte_pas=50) == 1000


def test_la_cible_reste_dans_la_plage_du_codeur():
    for actuel, zero in ((4090, 10), (5, 4090)):
        c = cible_de_rappel(actuel, zero, pas_max=200, zone_morte_pas=10)
        assert 0 <= c < 4096


# ------------------------------------------------------------------ couple minimum
def test_couple_minimum_des_la_sortie_de_la_zone_morte():
    """Sans plancher, le rappel est trop faible juste apres la zone morte pour vaincre
    le frottement : le bras y stagne (constate le 2026-09-05)."""
    _, c = commande(MORTE + 1e-6, DEMI, MORTE, CMAX, couple_min=20)
    assert c == 20                       # part du plancher, pas de zero
    _, c = commande(DEMI, DEMI, MORTE, CMAX, couple_min=20)
    assert c == CMAX                     # et atteint toujours le maximum


def test_le_plancher_ne_deborde_pas_dans_la_zone_morte():
    """Le couple doit rester EXACTEMENT nul au neutre, sinon le bras vibre."""
    for e in (0.0, MORTE / 2, MORTE):
        assert commande(e, DEMI, MORTE, CMAX, couple_min=20) == (0.0, 0)


def test_le_couple_croit_toujours_avec_l_ecart():
    vals = [commande(e, DEMI, MORTE, CMAX, couple_min=20)[1]
            for e in (0.11, 0.3, 0.5, 0.7, 0.9, 1.0)]
    assert vals == sorted(vals)
    assert vals[0] >= 20 and vals[-1] == CMAX


def test_plancher_superieur_au_plafond_est_borne():
    """Un reglage incoherent ne doit pas produire un couple superieur au maximum."""
    _, c = commande(0.5, DEMI, MORTE, 50, couple_min=200)
    assert c == 50
