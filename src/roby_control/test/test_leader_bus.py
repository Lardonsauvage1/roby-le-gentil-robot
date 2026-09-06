"""Tests de la couche bus du bras guide (US-018) — sans ROS ni materiel."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from roby_control.leader_bus import (  # noqa: E402
    RESOLUTION,
    LeaderBus,
    ServoNotResponding,
    decode_load,
    rad_to_steps,
    steps_to_rad,
    telemetry_warnings,
)

TWO_PI = 2.0 * math.pi


# ------------------------------------------------------------------ conversions
@pytest.mark.parametrize(
    "steps, rad",
    [
        (0, 0.0),                       # origine
        (1024, TWO_PI / 4),             # quart de tour
        (2048, math.pi),                # mi-course
        (3072, 3 * TWO_PI / 4),
        (4095, 4095 * TWO_PI / RESOLUTION),  # derniere valeur avant bouclage
    ],
)
def test_steps_to_rad(steps, rad):
    assert steps_to_rad(steps) == pytest.approx(rad)


def test_steps_to_rad_reste_dans_un_tour():
    for s in range(0, RESOLUTION, 37):
        assert 0.0 <= steps_to_rad(s) < TWO_PI


def test_bouclage_4096_revient_a_zero():
    """4096 pas = un tour complet : la valeur doit boucler, pas deborder."""
    assert steps_to_rad(RESOLUTION) == pytest.approx(0.0)
    assert steps_to_rad(RESOLUTION + 10) == pytest.approx(steps_to_rad(10))


def test_aller_retour_steps_rad():
    for s in (0, 1, 999, 2048, 3657, 4095):
        assert rad_to_steps(steps_to_rad(s)) == s


def test_rad_to_steps_ramene_dans_la_plage():
    assert rad_to_steps(0.0) == 0
    assert rad_to_steps(TWO_PI) == 0            # un tour complet -> origine
    assert rad_to_steps(-0.001) == RESOLUTION - 1  # negatif -> ramene par le haut
    assert 0 <= rad_to_steps(123.456) < RESOLUTION


# ------------------------------------------------------------------ erreurs typees
class _SyncMuet:
    """Transaction qui echoue : simule un bus muet."""

    def txRxPacket(self):
        return -1


class _SyncPartiel:
    """Transaction reussie mais un servo ne repond pas."""

    def __init__(self, presents):
        self.presents = presents

    def txRxPacket(self):
        return 0

    def isAvailable(self, sid, addr, size):
        return sid in self.presents

    def getData(self, sid, addr, size):
        return 1234


def _bus_factice(sync, ids=(1, 2, 3)):
    bus = LeaderBus(ids=ids, simulate=False)
    bus._sync = sync
    bus._COMM_SUCCESS = 0
    return bus


def test_bus_muet_leve_une_erreur_typee():
    bus = _bus_factice(_SyncMuet())
    with pytest.raises(ServoNotResponding):
        bus.read_positions()


def test_servo_absent_leve_une_erreur_et_ne_renvoie_pas_de_position():
    """Un servo manquant ne doit JAMAIS produire une position partielle ou fausse."""
    bus = _bus_factice(_SyncPartiel(presents={1, 3}))
    with pytest.raises(ServoNotResponding) as exc:
        bus.read_positions()
    assert "2" in str(exc.value)  # l'ID absent est nomme


def test_lecture_complete_retourne_tous_les_servos():
    bus = _bus_factice(_SyncPartiel(presents={1, 2, 3}))
    assert bus.read_positions() == {1: 1234, 2: 1234, 3: 1234}


# ------------------------------------------------------------------ mode simulation
def test_simulation_ne_touche_aucun_materiel():
    bus = LeaderBus(ids=(1, 2, 3, 4, 5, 6), simulate=True)
    bus.open()  # ne doit rien ouvrir
    pos = bus.read_positions()
    assert set(pos) == {1, 2, 3, 4, 5, 6}
    assert all(0 <= v < RESOLUTION for v in pos.values())
    assert bus.set_torque(True) == []
    assert bus.read_telemetry()[1]["torque_on"] is True
    assert bus.set_torque(False) == []
    assert bus.read_telemetry()[1]["torque_on"] is False
    bus.close()


def test_limite_de_couple_hors_plage_refusee():
    bus = LeaderBus(simulate=True)
    with pytest.raises(ValueError):
        bus.set_torque_limit(1001)
    with pytest.raises(ValueError):
        bus.set_torque_limit(-1)
    assert bus.set_torque_limit(500) == []


# ------------------------------------------------------------------ telemetrie
def test_charge_signee():
    assert decode_load(0) == 0
    assert decode_load(100) == 100
    assert decode_load((1 << 10) | 100) == -100


def test_avertissements_tension_et_temperature():
    assert telemetry_warnings(1, {"volt_dV": 74, "temp_C": 30}) == []
    assert "tension" in telemetry_warnings(1, {"volt_dV": 45, "temp_C": 30})[0]
    assert "temperature" in telemetry_warnings(1, {"volt_dV": 74, "temp_C": 60})[0]
    assert len(telemetry_warnings(1, {"volt_dV": None, "temp_C": None})) == 2


# ------------------------------------------------------------------ securite du couple
class _BusEspion:
    """Enregistre l'ordre des ecritures, pour verifier la sequence d'activation."""

    def __init__(self):
        self.ecritures = []

    def write(self, sid, reg, value):
        self.ecritures.append((sid, reg[0], value))
        return True


def test_activer_le_couple_fige_d_abord_la_cible():
    """La cible doit etre ecrite AVANT le couple, sinon le servo saute vers une cible
    perimee — dangereux quand l'operateur a la main sur le bras."""
    from roby_control.leader_bus import ADDR_GOAL_POSITION, ADDR_TORQUE_ENABLE

    espion = _BusEspion()
    bus = LeaderBus(ids=(1, 2), simulate=False)
    bus._sync = _SyncPartiel(presents={1, 2})
    bus._COMM_SUCCESS = 0
    bus._write_reg = espion.write

    assert bus.set_torque(True) == []
    regs = [e[1] for e in espion.ecritures]
    # Les 2 Goal_Position passent avant le premier Torque_Enable.
    assert regs.index(ADDR_TORQUE_ENABLE[0]) > max(
        i for i, r in enumerate(regs) if r == ADDR_GOAL_POSITION[0]
    )
    # Et la cible ecrite est bien la position MESUREE.
    for sid, reg, val in espion.ecritures:
        if reg == ADDR_GOAL_POSITION[0]:
            assert val == 1234


def test_couper_le_couple_n_ecrit_aucune_cible():
    from roby_control.leader_bus import ADDR_GOAL_POSITION

    espion = _BusEspion()
    bus = LeaderBus(ids=(1,), simulate=False)
    bus._COMM_SUCCESS = 0
    bus._write_reg = espion.write
    bus.set_torque(False)
    assert all(e[1] != ADDR_GOAL_POSITION[0] for e in espion.ecritures)


def test_bus_muet_empeche_d_activer_le_couple():
    """Si on ne peut pas lire la position, on ne peut pas figer la cible : on n'active
    donc PAS le couple plutot que de risquer un saut."""
    bus = _bus_factice(_SyncMuet())
    bus._write_reg = lambda *a: True
    with pytest.raises(ServoNotResponding):
        bus.set_torque(True)
