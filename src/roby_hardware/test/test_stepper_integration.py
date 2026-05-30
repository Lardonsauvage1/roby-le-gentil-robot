#!/usr/bin/env python3
"""Integration tests for roby_hardware stepper drivers.

Tests the ros2_control stack with real or mock hardware.
Run on Pi5 with motors connected, or on PC in mock mode.

Usage (Pi5, real hardware):
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42
    # Start the stack first:
    ros2 launch roby_hardware motor1_test.launch.py &
    sleep 15
    # Then run tests:
    python3 test_stepper_integration.py

Usage (PC, mock - via colcon test):
    colcon test --packages-select roby_hardware

Each test sends a trajectory goal and verifies the resulting joint_states.
Tests are designed to be safe: small movements, timeouts, position checks.
"""

import sys
import time
import unittest
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
POSITION_TOLERANCE = 0.01  # rad (~0.57 deg)
GOAL_TIMEOUT = 15.0  # seconds
SETTLE_TIME = 1.0  # seconds after goal to read position


class RobyTestNode(Node):
    def __init__(self):
        super().__init__("roby_integration_test")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._last_joint_state = None
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

    def _joint_state_cb(self, msg):
        self._last_joint_state = msg

    def wait_for_action_server(self, timeout=10.0):
        return self._action_client.wait_for_server(timeout_sec=timeout)

    def get_joint_positions(self):
        """Return dict of joint_name -> position."""
        if self._last_joint_state is None:
            return None
        return dict(zip(self._last_joint_state.name, self._last_joint_state.position))

    def send_goal_and_wait(self, positions, duration_sec=5.0):
        """Send a trajectory goal and wait for result.

        Args:
            positions: list of 5 joint positions [j1, j2, j3, j4, j5]
            duration_sec: time to reach the target

        Returns:
            (success: bool, error_string: str)
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points = [point]

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GOAL_TIMEOUT)

        if not future.done() or future.result() is None:
            return False, "Goal send timed out"

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, "Goal rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=GOAL_TIMEOUT)

        if not result_future.done() or result_future.result() is None:
            return False, "Result timed out"

        result = result_future.result().result
        return result.error_code == 0, result.error_string


class TestStepperIntegration(unittest.TestCase):
    """Integration tests for stepper motor control via ros2_control."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RobyTestNode()

        # Wait for action server
        assert cls.node.wait_for_action_server(timeout=30.0), (
            "Action server /arm_controller/follow_joint_trajectory not available. "
            "Is the stack running?"
        )

        # Wait for first joint_state message
        timeout = time.time() + 10.0
        while cls.node.get_joint_positions() is None and time.time() < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.5)
        assert cls.node.get_joint_positions() is not None, (
            "No /joint_states messages received"
        )

        # Return to home position at start
        cls._go_home()

    @classmethod
    def tearDownClass(cls):
        # Return to home position at end
        cls._go_home()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _go_home(cls):
        success, msg = cls.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        time.sleep(SETTLE_TIME)

    def _read_positions(self):
        """Spin and read current positions."""
        rclpy.spin_once(self.node, timeout_sec=0.5)
        return self.node.get_joint_positions()

    def _assert_position(self, joint_name, expected, tolerance=POSITION_TOLERANCE):
        pos = self._read_positions()
        self.assertIsNotNone(pos, "No joint_states")
        self.assertIn(joint_name, pos)
        self.assertAlmostEqual(
            pos[joint_name], expected, delta=tolerance,
            msg=f"{joint_name}: expected {expected:.4f}, got {pos[joint_name]:.4f}"
        )

    # ---- Test: action server responds ----

    def test_01_action_server_available(self):
        """The arm_controller action server is available."""
        self.assertTrue(
            self.node.wait_for_action_server(timeout=5.0)
        )

    # ---- Test: joint_states publishes all joints ----

    def test_02_joint_states_has_all_joints(self):
        """All 5 joints are present in /joint_states."""
        pos = self._read_positions()
        self.assertIsNotNone(pos)
        for name in JOINT_NAMES:
            self.assertIn(name, pos, f"Missing joint: {name}")

    # ---- Test: home position ----

    def test_03_home_position(self):
        """After going home, all joints are near 0."""
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=3.0
        )
        self.assertTrue(success, f"Home goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        for name in JOINT_NAMES:
            self._assert_position(name, 0.0, tolerance=0.05)

    # ---- Test: joint_1 (base) positive ----

    def test_10_joint1_positive(self):
        """joint_1 moves to +0.3 rad."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait(
            [0.3, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_1", 0.3)

    # ---- Test: joint_1 negative ----

    def test_11_joint1_negative(self):
        """joint_1 moves to -0.3 rad."""
        success, msg = self.node.send_goal_and_wait(
            [-0.3, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_1", -0.3)

    # ---- Test: joint_1 return to 0 ----

    def test_12_joint1_return(self):
        """joint_1 returns to 0 after movements."""
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_1", 0.0)

    # ---- Test: joint_2 (shoulder) positive = up ----

    def test_20_joint2_positive(self):
        """joint_2 moves to +0.2 rad (shoulder up)."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.2, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_2", 0.2)

    # ---- Test: joint_2 return ----

    def test_21_joint2_return(self):
        """joint_2 returns to 0."""
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_2", 0.0, tolerance=0.05)

    # ---- Test: joint_3 (elbow) negative = fold ----

    def test_30_joint3_negative(self):
        """joint_3 moves to -0.2 rad (elbow fold)."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, -0.2, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_3", -0.2)

    # ---- Test: joint_3 return ----

    def test_31_joint3_return(self):
        """joint_3 returns to 0."""
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_3", 0.0, tolerance=0.05)

    # ---- Test: coupling compensation ----

    def test_40_coupling_joint2_moves_joint3(self):
        """When joint_2 moves, joint_3 motor compensates (position includes coupling offset)."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.2, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        pos = self._read_positions()
        # joint_3 should have a negative offset due to coupling compensation
        # Expected: ~ -0.2 * (6000/45056) / (300/1408) ≈ -0.125 rad
        self.assertIsNotNone(pos)
        self.assertLess(pos["joint_3"], -0.05,
                        f"joint_3 should be negative due to coupling, got {pos['joint_3']:.4f}")
        self.assertGreater(pos["joint_3"], -0.25,
                           f"joint_3 coupling offset too large: {pos['joint_3']:.4f}")

    # ---- Test: combined 3-axis movement ----

    def test_50_combined_3axes(self):
        """Move all 3 stepper axes simultaneously."""
        self._go_home()
        target = [0.3, 0.2, -0.3, 0.0, 0.0]
        success, msg = self.node.send_goal_and_wait(target, duration_sec=5.0)
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_1", 0.3)
        self._assert_position("joint_2", 0.2)
        # joint_3 includes coupling offset from joint_2
        pos = self._read_positions()
        self.assertIsNotNone(pos)
        # joint_3 target = -0.3 + coupling from joint_2 at 0.2
        self.assertLess(pos["joint_3"], -0.3,
                        f"joint_3 should be more negative than -0.3 due to coupling")

    # ---- Test: combined return to 0 ----

    def test_51_combined_return(self):
        """Return to 0 after combined movement."""
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        # Larger tolerance for return due to open-loop drift
        for name in ["joint_1", "joint_2", "joint_3"]:
            self._assert_position(name, 0.0, tolerance=0.1)

    # ---- Test: mock joints 4-5 respond ----

    def test_60_mock_joints_respond(self):
        """Mock joints 4 and 5 follow commands."""
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.0, 0.0, 0.5, 0.3], duration_sec=3.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        time.sleep(SETTLE_TIME)
        self._assert_position("joint_4", 0.5)
        self._assert_position("joint_5", 0.3)
        # Return
        self._go_home()

    # ---- Test: safety - goal within limits ----

    def test_70_within_joint_limits(self):
        """Movements within URDF joint limits succeed."""
        # joint_2 limit: [-1.0, 1.2]
        success, msg = self.node.send_goal_and_wait(
            [0.0, 0.5, 0.0, 0.0, 0.0], duration_sec=5.0
        )
        self.assertTrue(success, f"Goal failed: {msg}")
        self._go_home()


if __name__ == "__main__":
    unittest.main(verbosity=2)
