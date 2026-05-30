#!/usr/bin/env python3
"""Trajectory relay: receives trajectories on a topic and forwards them
as action goals to the local arm_controller.

Workaround for CycloneDDS cross-machine action/service communication issues.
Topics work cross-machine, actions don't — this node bridges the gap.

Run on Pi5 alongside ros2_control:
    ros2 run roby_hardware trajectory_relay.py

Then from PC, publish a trajectory:
    ros2 topic pub --once /relay_trajectory trajectory_msgs/msg/JointTrajectory "..."
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory


class TrajectoryRelay(Node):
    def __init__(self):
        super().__init__("trajectory_relay")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._sub = self.create_subscription(
            JointTrajectory, "/relay_trajectory", self._on_trajectory, 10
        )
        self._current_goal = None
        self.get_logger().info("Trajectory relay ready — listening on /relay_trajectory")

    def _on_trajectory(self, msg):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("arm_controller action server not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg

        self.get_logger().info(
            f"Relaying trajectory: {len(msg.points)} points, "
            f"joints: {msg.joint_names}"
        )

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected by arm_controller")
            return
        self.get_logger().info("Goal accepted")
        self._current_goal = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        pass

    def _on_result(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info(f"Goal succeeded: {result.error_string}")
        else:
            self.get_logger().error(
                f"Goal failed: code={result.error_code}, {result.error_string}"
            )
        self._current_goal = None


def main():
    rclpy.init()
    node = TrajectoryRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
