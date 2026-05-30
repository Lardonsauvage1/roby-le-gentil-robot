"""Launch file for Roby hardware on Pi5.

Starts ros2_control_node with the real hardware plugin,
robot_state_publisher, and controller spawners.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="false",
        description="Use mock hardware instead of real GPIO/I2C",
    )

    # Get package paths
    roby_hw_share = FindPackageShare("roby_hardware")
    description_share = FindPackageShare("neuroneimitationcarote_description")

    # Build robot description (URDF) with real hardware plugin
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [roby_hw_share, "config", "roby_hardware.ros2_control.xacro"]
            ),
            # We need a wrapper xacro that includes both the robot URDF and our hardware config
            # For now, we reference the moveit config's master xacro with use_mock param
        ]
    )

    # Controller config
    controllers_yaml = PathJoinSubstitution(
        [roby_hw_share, "config", "roby_controllers.yaml"]
    )

    # Robot description from the moveit config package (includes URDF + ros2_control)
    moveit_share = FindPackageShare("neuroneimitationcarote_moveit_config")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [moveit_share, "config", "neuroneimitationcarote.urdf.xacro"]
            ),
        ]
    )

    # ros2_control_node (runs hardware plugin + controller_manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="both",
        emulate_tty=True,
    )

    # Robot state publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    # Controller spawners (delayed to let controller_manager start)
    jsb_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "30",
                ],
                output="both",
            )
        ],
    )

    arm_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "30",
                ],
                output="both",
            )
        ],
    )

    # Static TF: world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="both",
    )

    return LaunchDescription(
        [
            use_mock_arg,
            control_node,
            rsp_node,
            static_tf,
            jsb_spawner,
            arm_spawner,
        ]
    )
