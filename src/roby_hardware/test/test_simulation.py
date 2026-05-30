#!/usr/bin/env python3
"""Simulation tests for the MoveIt mock_components stack.

Tests the ros2_control simulation (mock_components/GenericSystem) on the PC.
No real hardware involved — all joints are simulated.

Usage:
    # Terminal 1: launch simulation
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine="true"/></Interfaces></General></Domain></CycloneDDS>'
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    unset GTK_PATH
    ros2 launch neuroneimitationcarote_moveit_config demo.launch.py

    # Terminal 2: run tests (wait 15s for stack startup)
    # same env exports as above
    python3 test_simulation.py
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
# Mock simulation has slight timing-based offsets
POSITION_TOLERANCE = 0.01  # rad (~0.57 deg)
GOAL_TIMEOUT = 15.0


class SimTestNode(Node):
    def __init__(self):
        super().__init__("sim_test_node")
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
        if self._last_joint_state is None:
            return None
        return dict(zip(self._last_joint_state.name, self._last_joint_state.position))

    def send_goal_and_wait(self, positions, duration_sec=3.0):
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


class TestSimulation(unittest.TestCase):
    """Tests for the mock_components simulation stack."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = SimTestNode()
        assert cls.node.wait_for_action_server(timeout=30.0), (
            "Action server not available. Is demo.launch.py running?"
        )
        timeout = time.time() + 10.0
        while cls.node.get_joint_positions() is None and time.time() < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.5)
        assert cls.node.get_joint_positions() is not None, "No /joint_states"
        cls._go_home()

    @classmethod
    def tearDownClass(cls):
        cls._go_home()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _go_home(cls):
        cls.node.send_goal_and_wait([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=3.0)
        time.sleep(0.5)

    def _read_positions(self):
        rclpy.spin_once(self.node, timeout_sec=0.5)
        return self.node.get_joint_positions()

    def _assert_position(self, joint_name, expected, tolerance=POSITION_TOLERANCE):
        pos = self._read_positions()
        self.assertIsNotNone(pos)
        self.assertIn(joint_name, pos)
        self.assertAlmostEqual(
            pos[joint_name], expected, delta=tolerance,
            msg=f"{joint_name}: expected {expected:.4f}, got {pos[joint_name]:.4f}"
        )

    # ---- Infrastructure ----

    def test_01_action_server(self):
        """Action server is available."""
        self.assertTrue(self.node.wait_for_action_server(timeout=5.0))

    def test_02_all_joints_present(self):
        """All 5 joints in /joint_states."""
        pos = self._read_positions()
        for name in JOINT_NAMES:
            self.assertIn(name, pos)

    def test_03_home_position(self):
        """Home position: all joints at 0."""
        self._go_home()
        for name in JOINT_NAMES:
            self._assert_position(name, 0.0)

    # ---- Individual joints ----

    def test_10_joint1_positive(self):
        """joint_1 to +1.0 rad."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait([1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        self._assert_position("joint_1", 1.0)

    def test_11_joint1_negative(self):
        """joint_1 to -1.0 rad."""
        success, msg = self.node.send_goal_and_wait([-1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        self._assert_position("joint_1", -1.0)

    def test_20_joint2(self):
        """joint_2 to +0.5 rad."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait([0.0, 0.5, 0.0, 0.0, 0.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        self._assert_position("joint_2", 0.5)

    def test_30_joint3(self):
        """joint_3 to -1.0 rad."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait([0.0, 0.0, -1.0, 0.0, 0.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        self._assert_position("joint_3", -1.0)

    def test_40_joint4(self):
        """joint_4 to +1.5 rad."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait([0.0, 0.0, 0.0, 1.5, 0.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        self._assert_position("joint_4", 1.5)

    def test_50_joint5(self):
        """joint_5 to +1.0 rad."""
        self._go_home()
        success, msg = self.node.send_goal_and_wait([0.0, 0.0, 0.0, 0.0, 1.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        self._assert_position("joint_5", 1.0)

    # ---- Combined ----

    def test_60_all_joints_combined(self):
        """Move all 5 joints simultaneously."""
        self._go_home()
        target = [0.5, 0.3, -0.5, 1.0, 0.5]
        success, msg = self.node.send_goal_and_wait(target)
        self.assertTrue(success, msg)
        time.sleep(0.5)
        for name, expected in zip(JOINT_NAMES, target):
            self._assert_position(name, expected)

    def test_61_return_home_exact(self):
        """Return to 0 is exact in simulation (no drift)."""
        success, msg = self.node.send_goal_and_wait([0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertTrue(success, msg)
        time.sleep(0.5)
        for name in JOINT_NAMES:
            self._assert_position(name, 0.0)

    # ---- Multi-point trajectory ----

    def test_70_multi_point_trajectory(self):
        """Execute a multi-point trajectory."""
        self._go_home()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        points = [
            ([0.3, 0.0, 0.0, 0.0, 0.0], 2.0),
            ([0.3, 0.2, -0.3, 0.0, 0.0], 4.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0], 6.0),
        ]
        for positions, t in points:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(t), nanosec=0)
            goal.trajectory.points.append(point)

        future = self.node._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=GOAL_TIMEOUT)
        goal_handle = future.result()
        self.assertTrue(goal_handle.accepted, "Multi-point goal rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=GOAL_TIMEOUT)
        result = result_future.result().result
        self.assertEqual(result.error_code, 0, result.error_string)
        time.sleep(0.5)
        # Should be at last point (home)
        for name in JOINT_NAMES:
            self._assert_position(name, 0.0)

    # ---- Named poses from SRDF ----

    def test_80_transport_pose(self):
        """Move to transport pose [0, -0.3558, 0.6097, 0, 1.3702]."""
        self._go_home()
        transport = [0.0, -0.3558, 0.6097, 0.0, 1.3702]
        success, msg = self.node.send_goal_and_wait(transport, duration_sec=3.0)
        self.assertTrue(success, msg)
        time.sleep(0.5)
        for name, expected in zip(JOINT_NAMES, transport):
            self._assert_position(name, expected)
        self._go_home()


if __name__ == "__main__":
    unittest.main(verbosity=2)
