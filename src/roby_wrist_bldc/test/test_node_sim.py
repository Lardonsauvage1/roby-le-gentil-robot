"""Integration ROS du noeud wrist_bldc contre la carte simulee (~15 s).

Domaine DDS isole et localhost uniquement : ne parle JAMAIS au robot (domaine 42).
"""

import os
import threading
import time

import pytest

# 87 par defaut (88 : pont ros2_control, 89 : teleop du bras guide -- colcon test lance
# les paquets en parallele). ROBY_TEST_DOMAIN_ID permet d'isoler des series lancees en
# parallele ; 42 (vraie stack) et 43 (simulation) sont refuses.
_DOMAINE = os.environ.get("ROBY_TEST_DOMAIN_ID", "87")
if _DOMAINE in ("42", "43"):
    raise RuntimeError("domaine DDS %s reserve : jamais pour un test" % _DOMAINE)
os.environ["ROS_DOMAIN_ID"] = _DOMAINE
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool, Float64, Float64MultiArray  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from roby_wrist_bldc.node import (  # noqa: E402
    FLAG_FAULT,
    FLAG_FOLLOWING,
    FLAG_HOMED,
    FLAG_LINK_OK,
    WristBldcNode,
)

NEST = 0.2009


class Harness:
    def __init__(self):
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx)
        self.node = WristBldcNode(
            context=self.ctx,
            namespace="roby",
            parameter_overrides=[
                Parameter("simulate", value=True),
                Parameter("nest_position", value=NEST),
                Parameter(
                    "tracking_error_limit", value=0.0
                ),  # on teste le defaut CARTE
            ],
        )
        self.node._sim.stall_time_s = 0.3
        self.client = rclpy.create_node("test_client", context=self.ctx)
        self.state = None
        self.fault = None
        self.client.create_subscription(
            Float64MultiArray, "/roby/wrist_bldc/state", self._on_state, 10
        )
        self.client.create_subscription(
            Bool, "/roby/wrist_bldc/fault", self._on_fault, 10
        )
        self.cmd = self.client.create_publisher(Float64, "/roby/wrist_bldc/command", 10)
        self.reset = self.client.create_client(Trigger, "/roby/wrist_bldc/reset_fault")
        self.executor = MultiThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.executor.add_node(self.client)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_state(self, msg):
        self.state = list(msg.data)

    def _on_fault(self, msg):
        self.fault = msg.data

    def flags(self):
        return 0 if self.state is None else int(self.state[2])

    def position(self):
        return self.state[0]

    def stream(self, start, end, seconds, rate=100.0):
        """Flux de consignes type JointTrajectoryController (interpolation lineaire)."""
        n = max(1, int(seconds * rate))
        for i in range(n + 1):
            self.cmd.publish(Float64(data=start + (end - start) * i / n))
            time.sleep(1.0 / rate)

    def hold(self, value, seconds, rate=100.0):
        self.stream(value, value, seconds, rate)

    def hold_until(self, value, predicate, timeout, rate=100.0):
        """Maintient la consigne `value` (flux continu, comme ros2_control) jusqu'a ce
        que `predicate` soit vrai, au plus `timeout` s.

        Attendre SANS publier fausse le test : au bout de command_timeout le noeud
        conclut que ros2_control s'est arrete et re-cible son point d'arret. Sur une
        machine chargee, les durees fixes (hold puis assert) echouaient pour cette
        raison (2026-09-13 : 8 echecs sur 8 series paralleles).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            self.cmd.publish(Float64(data=value))
            time.sleep(1.0 / rate)
        return predicate()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.client.destroy_node()
        rclpy.shutdown(context=self.ctx)


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def h():
    harness = Harness()
    yield harness
    harness.close()


def test_node_homes_follows_faults_and_resumes_safely(h):
    # 1. recalage au nid automatique au demarrage
    ready = FLAG_LINK_OK | FLAG_HOMED | FLAG_FOLLOWING
    assert wait_for(lambda: h.flags() & ready == ready, timeout=8.0)
    assert abs(h.position() - NEST) < 1e-3

    # 2. suit un flux de consignes 100 Hz
    h.hold(NEST, 0.2)
    h.stream(NEST, 0.5, 1.0)
    # Le limiteur suit le flux exactement puis, a l'arret net de la rampe, depasse
    # d'environ v^2/2a (~0,03 rad) avant de revenir : on attend la convergence.
    assert h.hold_until(0.5, lambda: abs(h.position() - 0.5) < 0.01, timeout=3.0)

    # 3. obstacle => defaut carte, consignes ignorees, defaut expose
    h.node._sim.set_obstacle(-10.0, 0.6)
    h.stream(0.5, 0.9, 1.0)
    assert h.hold_until(0.9, lambda: h.flags() & FLAG_FAULT, timeout=5.0)
    assert wait_for(lambda: h.fault is True, timeout=1.0)
    assert not h.flags() & FLAG_FOLLOWING

    # 4. reset : la vieille consigne de maintien (0.9) n'est PAS rejointe
    h.node._sim.clear_obstacle()
    assert h.reset.wait_for_service(timeout_sec=2.0)
    future = h.reset.call_async(Trigger.Request())
    assert wait_for(future.done, timeout=2.0) and future.result().success
    h.hold(0.9, 1.0)
    assert (
        abs(h.position() - 0.6) < 0.02
    ), "l'axe ne doit pas repartir seul vers l'obstacle"
    assert wait_for(lambda: h.fault is False, timeout=1.0)

    # 5. une nouvelle trajectoire (partant de la mesure) est suivie
    here = h.position()
    h.stream(here, 0.3, 1.0)
    assert h.hold_until(0.3, lambda: abs(h.position() - 0.3) < 0.01, timeout=3.0)


def test_command_timeout_stops_on_ramp(h):
    ready = FLAG_LINK_OK | FLAG_HOMED | FLAG_FOLLOWING
    assert wait_for(lambda: h.flags() & ready == ready, timeout=8.0)
    h.hold(NEST, 0.1)
    # un seul saut de consigne puis plus rien (ros2_control arrete)
    h.cmd.publish(Float64(data=1.5))
    assert wait_for(lambda: h.node.driver.get_status().target == 1.5, timeout=1.0)

    def stopped():
        s = h.node.driver.get_status()
        return s.target is not None and s.target < 1.4 and s.board_target == s.target

    # la rampe est re-ciblee sur son point d'arret (le moteur reste asservi)
    assert wait_for(stopped, timeout=3.0)
    assert wait_for(
        lambda: abs(h.position() - h.node.driver.get_status().target) < 0.01,
        timeout=3.0,
    ), "le moteur reste asservi sur le point d'arret"
    final = h.position()
    assert NEST < final < 1.4, "arret sur rampe avant la cible abandonnee"


def test_reset_ignores_stale_hold_without_waiting_for_a_state_tick(h):
    """Regression 2026-09-13 : le verrou de reprise n'etait pose qu'au tick de
    publication suivant ; une consigne arrivee juste apres reset_fault etait suivie
    et l'axe repartait vers l'obstacle."""
    ready = FLAG_LINK_OK | FLAG_HOMED | FLAG_FOLLOWING
    assert wait_for(lambda: h.flags() & ready == ready, timeout=8.0)
    h.stream(NEST, 0.5, 1.0)
    h.node._sim.set_obstacle(-10.0, 0.6)
    h.stream(0.5, 0.9, 1.0)
    assert h.hold_until(0.9, lambda: h.flags() & FLAG_FAULT, timeout=5.0)
    h.node._sim.clear_obstacle()
    # La consigne de maintien du controleur est bien la derniere recue (en service,
    # ros2_control la repete a 100 Hz depuis des secondes quand on appelle reset).
    assert h.hold_until(0.9, lambda: h.node._last_command == 0.9, timeout=2.0)
    # exactement l'action du service, puis la vieille consigne DANS LA FOULEE
    h.node._trigger_actions["~/reset_fault"]()
    h.node._on_command(Float64(data=0.9))
    target = h.node.driver.get_status().target
    assert target is None or abs(target - 0.9) > 0.05, "vieille consigne suivie"
