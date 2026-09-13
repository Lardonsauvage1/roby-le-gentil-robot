#!/usr/bin/env python3
"""Integration SANS MATERIEL du pont BLDC : JTC -> RobySystem (joint_5 bldc) -> wrist_bldc -> carte simulee.

Verifie :
  1. activation : joint_5 part de la mesure (nid) ;
  2. une trajectoire FollowJointTrajectory sur joint_5 est suivie, /joint_states = mesure ;
  3. noeud wrist_bldc tue : le plugin bascule en open-loop sans desactiver le bras.

Domaine DDS 88, localhost : ne parle JAMAIS au robot (domaine 42).

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -m pytest -q src/roby_hardware/test/test_bldc_bridge_sim.py      (~40 s)
"""

import os
import signal
import subprocess
import tempfile
import threading
import time

import pytest

os.environ["ROS_DOMAIN_ID"] = "88"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["CYCLONEDDS_URI"] = (
    "<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine=\"true\"/>"
    "</Interfaces></General></Domain></CycloneDDS>"
)

rclpy = pytest.importorskip("rclpy")
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "bldc_bridge_sim", "bldc_bridge_sim.launch.py")
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
NEST = 0.2009


def wait_for(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture(scope="module")
def stack():
    log = tempfile.NamedTemporaryFile("w+", prefix="bldc_bridge_sim_", suffix=".log", delete=False)
    proc = subprocess.Popen(
        ["ros2", "launch", LAUNCH], stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    yield proc, log.name
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    print(f"log de la stack : {log.name}")


@pytest.fixture(scope="module")
def client(stack):
    rclpy.init()
    node = rclpy.create_node("bldc_bridge_test")
    data = {"js": None, "wrist": None}
    node.create_subscription(JointState, "/joint_states", lambda m: data.__setitem__("js", m), 10)
    node.create_subscription(
        Float64MultiArray, "/roby/wrist_bldc/state", lambda m: data.__setitem__("wrist", list(m.data)), 10
    )
    action = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node, data, action
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def joint5(data):
    js = data["js"]
    if js is None or "joint_5" not in js.name:
        return None
    return js.position[js.name.index("joint_5")]


def send(action, target5, seconds):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0, 0.0, 0.0, target5]
    point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
    goal.trajectory.points = [point]
    future = action.send_goal_async(goal)
    assert wait_for(future.done, 5.0)
    handle = future.result()
    assert handle.accepted
    result = handle.get_result_async()
    assert wait_for(result.done, seconds + 10.0)
    return result.result().result


def test_1_activation_starts_from_measured_nest(client):
    _, data, action = client
    assert wait_for(lambda: joint5(data) is not None, 30.0), "pas de /joint_states"
    assert abs(joint5(data) - NEST) < 2e-3
    assert action.wait_for_server(timeout_sec=20.0)


def test_2_trajectory_is_followed_with_real_feedback(client):
    _, data, action = client
    samples = []
    stop = threading.Event()

    def sample():
        while not stop.is_set():
            value = joint5(data)
            if value is not None:
                samples.append(value)
            time.sleep(0.02)

    sampler = threading.Thread(target=sample, daemon=True)
    sampler.start()
    result = send(action, 0.6, 2.0)
    time.sleep(0.8)
    stop.set()
    assert result.error_code == 0, result.error_string
    final = joint5(data)
    assert abs(final - 0.6) < 0.02, f"joint_5 = {final:.4f}"
    assert abs(data["wrist"][0] - 0.6) < 0.02
    steps = [b - a for a, b in zip(samples, samples[1:])]
    assert min(steps) > -0.005, "pas de retour en arriere pendant la montee"


def test_3_wrist_node_loss_keeps_arm_active(stack, client):
    proc, log_path = stack
    _, data, _ = client
    held = joint5(data)
    subprocess.run(["pkill", "-INT", "-f", "wrist_bldc_node"], check=False)
    time.sleep(1.5)
    stamp = data["js"].header.stamp
    time.sleep(0.5)
    assert data["js"].header.stamp != stamp, "/joint_states doit continuer (bras toujours actif)"
    assert abs(joint5(data) - held) < 0.02, "open-loop : joint_5 = consigne de maintien"
    with open(log_path, encoding="utf-8", errors="replace") as fh:
        log = fh.read()
    assert "mesure BLDC PERDUE" in log
    assert "mesure BLDC recue" in log
