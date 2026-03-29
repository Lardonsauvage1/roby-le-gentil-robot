#!/usr/bin/env python3
"""
Trajectoire en boucle pour le bras neuroneimitationcarote.
Le robot enchaîne plusieurs poses en simulation, indéfiniment.

Lancer d'abord la simulation :
    ~/ros2_ws/launch_sim.sh

Puis dans un autre terminal :
    env -u GTK_PATH -u CYCLONEDDS_URI \
      bash -c 'source /opt/ros/jazzy/setup.bash && \
      source ~/ros2_ws/install/setup.bash && \
      export ROS_DOMAIN_ID=42 && \
      python3 ~/ros2_ws/src/neuroneimitationcarote_moveit_config/scripts/loop_trajectory.py'
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

# Limites joints (pour référence) :
# joint_1: [-3.14, 3.14]   (base, rotation Z)
# joint_2: [-1.0, 1.2]     (épaule, rotation Y)
# joint_3: [-3.0, 0.65]    (coude, rotation Y)
# joint_4: [-3.14, 3.14]   (poignet rotation, X)
# joint_5: [-1.6, 1.6]     (poignet flexion, Y)

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

# Séquence de poses nommées
POSES = [
    ("home",          [0.0,    0.0,     0.0,    0.0,   0.0]),
    ("bras levé",     [0.0,   -0.8,     0.0,    0.0,   0.0]),
    ("coude plié",    [0.0,   -0.8,    -1.5,    0.0,   0.0]),
    ("rotation base", [1.5,   -0.8,    -1.5,    0.0,   0.0]),
    ("poignet",       [1.5,   -0.8,    -1.5,    1.5,   1.0]),
    ("transport",     [0.0,   -0.3558,  0.6097,  0.0,   1.3702]),
    ("gauche haut",   [-1.2,  -0.9,    -0.5,    0.0,   0.0]),
    ("gauche bas",    [-1.2,   0.5,    -2.0,    0.0,  -0.5]),
    ("centre bas",    [0.0,    0.5,    -2.0,    0.0,  -0.5]),
    ("home",          [0.0,    0.0,     0.0,    0.0,   0.0]),
]


class LoopTrajectory(Node):
    def __init__(self):
        super().__init__("loop_trajectory")
        self._client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )
        self.get_logger().info("Attente du serveur arm_controller...")
        self._client.wait_for_server()
        self.get_logger().info("Connecté!")

    def send_trajectory(self, positions_list, names_list, duration_per_point=3.0):
        """Envoie une trajectoire multi-points."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        for i, (name, positions) in enumerate(positions_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(
                sec=int((i + 1) * duration_per_point),
                nanosec=0
            )
            goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Envoi trajectoire: {len(positions_list)} poses, "
            f"durée totale: {len(positions_list) * duration_per_point:.0f}s"
        )

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Trajectoire rejetée!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result and result.result.error_code == 0:
            self.get_logger().info("Trajectoire terminée avec succès!")
            return True
        else:
            code = result.result.error_code if result else "timeout"
            self.get_logger().error(f"Erreur trajectoire: {code}")
            return False


def main():
    rclpy.init()
    node = LoopTrajectory()

    cycle = 0
    try:
        while rclpy.ok():
            cycle += 1
            node.get_logger().info(
                f"{'='*50}\n"
                f"  CYCLE {cycle} — {len(POSES)} poses\n"
                f"{'='*50}"
            )

            for i, (name, pos) in enumerate(POSES):
                node.get_logger().info(
                    f"  [{i+1}/{len(POSES)}] → {name}  "
                    f"{[round(p, 2) for p in pos]}"
                )

            success = node.send_trajectory(POSES, JOINT_NAMES, duration_per_point=3.0)

            if not success:
                node.get_logger().warn("Échec, nouvel essai dans 2s...")
                time.sleep(2.0)
            else:
                node.get_logger().info("Pause 1s avant prochain cycle...")
                time.sleep(1.0)

    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
