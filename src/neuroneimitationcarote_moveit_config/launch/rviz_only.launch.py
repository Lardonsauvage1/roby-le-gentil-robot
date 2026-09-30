"""Launch RViz only for visualization (no move_group, no ros2_control).

Use this on the PC when move_group and ros2_control both run on the Pi5.
RViz subscribes to /joint_states and /tf via DDS to display the robot state.

Usage (PC):
    export CYCLONEDDS_URI=file:///home/sam/cyclone_config.xml   # pair Pi5 : deploy/pc/
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42
    unset GTK_PATH
    ros2 launch neuroneimitationcarote_moveit_config rviz_only.launch.py
"""

import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Refus du domaine 42 (2026-09-13, spec-point-entree-unique-lancement) : ce lancement
    # publie l'URDF MOCK du moveit_config ; a cote du vrai bras, c'est la course au mock (steppers morts, BUG-008).
    if os.environ.get("ROS_DOMAIN_ID") == "42":
        raise RuntimeError(
            "rviz_only.launch.py refuse sur le domaine 42 (vrai robot) : il publie l'URDF MOCK du moveit_config. "
            "Vrai robot : `roby up --nid-confirme` ; simulation : `roby up --sim`."
        )
    moveit_config = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config"
    ).to_moveit_configs()

    ld = LaunchDescription()

    # PAS de robot_state_publisher cote PC : il publierait l'URDF mock
    # (FakeSystem) sur /robot_description et polluerait le topic partage ->
    # le ros2_control_node du Pi chargerait le mock au lieu de RobySystem.
    # RViz recupere robot_description + TF publies par le RSP du Pi (RobySystem).
    # for action in generate_rsp_launch(moveit_config).entities:
    #     ld.add_action(action)

    # NO move_group — it runs on Pi5
    # NO ros2_control — it runs on Pi5

    # RViz with MoveIt plugin (visualization only)
    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(action)

    # Static TF: world -> base_link
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        output="screen",
    ))

    return ld
