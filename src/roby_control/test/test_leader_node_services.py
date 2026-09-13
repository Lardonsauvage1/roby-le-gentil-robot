"""leader_node : une panne du bus pendant un service ne tue pas le noeud (revue 2026-09-13).

Bus en mode `simulate` (aucun materiel), domaine DDS isole.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

DOMAINE = 90          # ni 42 (vraie stack) ni 43 (simulation), ni 87-89 (autres tests)
os.environ["ROS_DOMAIN_ID"] = str(DOMAINE)

import rclpy  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from roby_control.leader_bus import ServoNotResponding  # noqa: E402
from roby_control.leader_node import LeaderNode  # noqa: E402


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init(args=["--ros-args", "-p", "simulate:=true"], domain_id=DOMAINE)
    yield
    rclpy.shutdown()


def test_panne_du_bus_pendant_un_service_coupe_le_couple_sans_tuer_le_noeud():
    n = LeaderNode()
    try:
        appels = []

        def bus_en_panne(actif, ids=None):
            appels.append(bool(actif))
            if actif:
                raise ServoNotResponding("servo 3 muet (lecture groupee)")
            return []

        n.bus.set_torque = bus_en_panne
        # appel direct du callback enregistre : exactement ce que ferait rclpy
        r = n.srv_torque.callback(SetBool.Request(data=True), SetBool.Response())
        assert r.success is False and "couple coupe" in r.message
        assert appels == [True, False]          # tentative, puis coupure de securite
        r = n.srv_maintien.callback(SetBool.Request(data=True), SetBool.Response())
        assert r.success is False and n.maintien is False
    finally:
        n.shutdown()
        n.destroy_node()
