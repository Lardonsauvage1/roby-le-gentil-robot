"""Launch MoveIt planning + RViz only (no ros2_control).

Use this when ros2_control runs on the Pi5 with real hardware.
MoveIt connects to the arm_controller and joint_state_broadcaster
running on the Pi5 via DDS.

Usage (PC):
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine="true"/></Interfaces></General></Domain></CycloneDDS>'
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42
    unset GTK_PATH
    ros2 launch neuroneimitationcarote_moveit_config moveit_only.launch.py
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config"
    ).to_moveit_configs()

    # Joint velocity/acceleration limits
    limits = moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
    for joint in ["joint_1", "joint_2", "joint_3"]:
        limits[joint] = {
            "has_velocity_limits": True,
            "max_velocity": 1.0,
            "has_acceleration_limits": True,
            "max_acceleration": 1.0,
        }
    for joint in ["joint_4", "joint_5"]:
        limits[joint] = {
            "has_velocity_limits": True,
            "max_velocity": 2.0,
            "has_acceleration_limits": True,
            "max_acceleration": 2.0,
        }

    ld = LaunchDescription()

    # Robot state publisher (URDF + TF)
    for action in generate_rsp_launch(moveit_config).entities:
        ld.add_action(action)

    # NO ros2_control_node — it runs on Pi5
    # NO controller spawners — they run on Pi5

    # MoveIt move_group (trajectory planning)
    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)

    # RViz with MoveIt plugin
    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(action)

    # Static TF: world → base_link
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        output="screen",
    ))

    return ld
