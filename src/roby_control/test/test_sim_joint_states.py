"""Un bras SIMULE ne publie jamais /joint_states a cote d'un vrai robot (BUG-008).

Tests unitaires de GardeJointStates, sans ROS : noeud et publisher factices, horloge
maitrisee.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from roby_control import sim_joint_states as sjs  # noqa: E402


class Horloge:
    t = 100.0

    @classmethod
    def monotonic(cls):
        return cls.t


class Logger:
    def __init__(self):
        self.erreurs, self.infos = [], []

    def error(self, m):
        self.erreurs.append(m)

    def info(self, m):
        self.infos.append(m)


class Noeud:
    def __init__(self):
        self.n = 1                     # lui-meme
        self.log = Logger()

    def count_publishers(self, topic):
        assert topic == "/joint_states"
        return self.n

    def get_logger(self):
        return self.log


class Pub:
    def __init__(self):
        self.envoyes = []

    def publish(self, m):
        self.envoyes.append(m)


def _garde(monkeypatch, attente=2.0):
    monkeypatch.setattr(sjs, "time", Horloge)
    Horloge.t = 100.0
    noeud, pub = Noeud(), Pub()
    return noeud, pub, sjs.GardeJointStates(noeud, pub, attente_s=attente, periode_s=0.5)


def test_seul_il_publie_apres_la_decouverte(monkeypatch):
    noeud, pub, g = _garde(monkeypatch)
    assert g.publier("m0") is False              # decouverte DDS en cours : silence
    Horloge.t += 2.1
    assert g.publier("m1") is True
    assert pub.envoyes == ["m1"]


def test_a_cote_d_un_autre_publisher_il_se_tait_et_le_dit_une_fois(monkeypatch):
    noeud, pub, g = _garde(monkeypatch)
    noeud.n = 2                                   # le joint_state_broadcaster du Pi5
    for _ in range(10):
        Horloge.t += 0.6
        assert g.publier("m") is False
    assert pub.envoyes == []
    assert len(noeud.log.erreurs) == 1 and "BUG-008" in noeud.log.erreurs[0]


def test_le_robot_apparait_en_cours_de_route(monkeypatch):
    noeud, pub, g = _garde(monkeypatch)
    Horloge.t += 2.1
    assert g.publier("a") is True
    noeud.n = 2
    Horloge.t += 0.6                              # detecte au plus une periode apres
    assert g.publier("b") is False
    noeud.n = 1                                   # la vraie stack s'arrete
    Horloge.t += 0.6
    assert g.publier("c") is True
    assert pub.envoyes == ["a", "c"]
