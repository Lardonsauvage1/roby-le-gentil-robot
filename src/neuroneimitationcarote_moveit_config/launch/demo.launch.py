from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config"
    ).to_moveit_configs()

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

    # Robot state publisher (publie URDF + TF)
    for action in generate_rsp_launch(moveit_config).entities:
        ld.add_action(action)

    # ros2_control node (mock hardware pour simulation)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    )
    ld.add_action(ros2_control_node)

    # Spawn controllers après démarrage du controller_manager
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[
                            Node(
                                package="controller_manager",
                                executable="spawner",
                                arguments=[
                                    "joint_state_broadcaster",
                                    "--controller-manager-timeout", "30",
                                ],
                                output="screen",
                            ),
                            Node(
                                package="controller_manager",
                                executable="spawner",
                                arguments=[
                                    "arm_controller",
                                    "--controller-manager-timeout", "30",
                                ],
                                output="screen",
                            ),
                        ],
                    ),
                ],
            )
        )
    )

    # MoveIt move_group (planification de mouvement)
    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)

    # RViz avec plugin MoveIt
    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(action)

    # TF statique : world → base_link (robot fixe au sol)
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        output="screen",
    ))

    return ld
