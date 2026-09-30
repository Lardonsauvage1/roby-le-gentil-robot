"""RViz viewer only — for monitoring the robot from the PC.

All computation (ros2_control, move_group, robot_state_publisher) runs on Pi5.
This launch file starts ONLY rviz2 which subscribes to /robot_description,
/joint_states and /tf via DDS.  No duplicate nodes.

Usage (PC):
    ros2 launch neuroneimitationcarote_moveit_config rviz_viewer.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Refus du domaine 42 (2026-09-13, spec-point-entree-unique-lancement) : ce lancement
    # publie l'URDF MOCK du moveit_config ; a cote du vrai bras, c'est la course au mock (steppers morts, BUG-008).
    if os.environ.get("ROS_DOMAIN_ID") == "42":
        raise RuntimeError(
            "rviz_viewer.launch.py refuse sur le domaine 42 (vrai robot) : il publie l'URDF MOCK du moveit_config. "
            "Vrai robot : `roby up --nid-confirme` ; simulation : `roby up --sim`."
        )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("neuroneimitationcarote_moveit_config"), "config", "viewer.rviz"]
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
