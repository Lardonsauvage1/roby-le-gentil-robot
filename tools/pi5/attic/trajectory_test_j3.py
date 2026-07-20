#!/usr/bin/env python3
"""Test isole sur joint_3 (coude).

Le couplage axe 2/3 fait que physiquement, quand on commande joint_3,
le motor_3 bouge ET motor_2 reste statique (joint_2 stable a 0).
Donc joint_2_meas devrait rester ~stable durant le test (modulo bruit
du capteur).

Sequence (~9s) :
  T+0 : warmup 2s
  T+2 : goal joint_3 = -0.1 rad sur 1.5s
  T+5 : goal joint_3 = 0.0 rad sur 1.5s
  T+9 : fin
"""

import csv
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

CSV_PATH = "/tmp/trajectory_test_j3.csv"
ALL_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
LOG_JOINTS = ["joint_1", "joint_2", "joint_3"]


def _extract(msg, joint_names):
    if not msg or not msg.name:
        return None
    name_to_pos = dict(zip(msg.name, msg.position))
    if not all(j in name_to_pos for j in joint_names):
        return None
    return [name_to_pos[j] for j in joint_names]


class TrajectoryTestJ3(Node):
    def __init__(self):
        super().__init__("trajectory_test_j3")
        self.create_subscription(JointState, "/joint_states", self._on_cmd, 10)
        self.create_subscription(JointState, "/joint_states_measured", self._on_meas, 10)
        self._ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self._last_cmd = None
        self._last_meas = None
        self._csv_file = open(CSV_PATH, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["t_s",
             "j1_cmd", "j2_cmd", "j3_cmd",
             "j1_meas", "j2_meas", "j3_meas",
             "delta_j1_deg", "delta_j2_deg", "delta_j3_deg"]
        )
        self._t0 = time.time()
        self.create_timer(0.02, self._log_tick)

    def _on_cmd(self, msg):
        v = _extract(msg, LOG_JOINTS)
        if v is not None:
            self._last_cmd = v

    def _on_meas(self, msg):
        v = _extract(msg, LOG_JOINTS)
        if v is not None:
            self._last_meas = v

    def _log_tick(self):
        if self._last_cmd is None or self._last_meas is None:
            return
        t = time.time() - self._t0
        deltas = [math.degrees(self._last_meas[i] - self._last_cmd[i]) for i in range(3)]
        self._csv.writerow(
            [f"{t:.4f}"]
            + [f"{v:.6f}" for v in self._last_cmd]
            + [f"{v:.6f}" for v in self._last_meas]
            + [f"{d:.4f}" for d in deltas]
        )

    def send_goal(self, joint_3_target_rad, duration_s):
        if not self._ac.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("action server not available !")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ALL_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, float(joint_3_target_rad), 0.0, 0.0]
        sec = int(duration_s)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration_s - sec) * 1e9)
        goal.trajectory.points = [point]
        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected !")
            return False
        self.get_logger().info(f"goal accepte : joint_3={joint_3_target_rad:+.3f} rad sur {duration_s}s")
        return True

    def close(self):
        self._csv_file.close()
        self.get_logger().info(f"CSV ferme : {CSV_PATH}")


def main():
    rclpy.init()
    node = TrajectoryTestJ3()
    try:
        node.get_logger().info("=== T+0 : warmup (2s) ===")
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info("=== T+2 : goal joint_3 = -0.1 rad (1.5s) ===")
        node.send_goal(-0.1, 1.5)

        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info("=== T+5 : goal joint_3 = 0.0 rad (1.5s) ===")
        node.send_goal(0.0, 1.5)

        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info("=== T+9 : fin ===")
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
