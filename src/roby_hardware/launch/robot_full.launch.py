"""Full robot launch: ros2_control (real hardware) + MoveIt move_group.

Run on Pi5:
    export ROS_DOMAIN_ID=42
    ros2 launch roby_hardware robot_full.launch.py

The PC runs RViz separately for visualization:
    ros2 launch neuroneimitationcarote_moveit_config moveit_only.launch.py
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    roby_hw_share = FindPackageShare("roby_hardware")

    # URDF with steppers_only hardware config (joints 1-3 real, 4-5 mock)
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [roby_hw_share, "config", "roby_motor1_test.urdf.xacro"]
            ),
        ]
    )

    controllers_yaml = PathJoinSubstitution(
        [roby_hw_share, "config", "roby_controllers.yaml"]
    )

    robot_description_param = ParameterValue(robot_description, value_type=str)

    # --- ros2_control stack ---

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_param}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_param},
            controllers_yaml,
        ],
        output="both",
        emulate_tty=True,
    )

    jsb_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
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
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
                ],
                output="both",
            )
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="both",
    )

    # --- MoveIt move_group ---

    moveit_config = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config"
    ).to_moveit_configs()

    # Joint velocity/acceleration limits
    limits = moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
    # Limites prudentes : evite l'overshoot dynamique en boucle ouverte (les
    # drivers stepper ne freinent pas activement, l'inertie continue apres la
    # commande). 0.2 rad/s = 11.5 deg/s, 0.5 rad/s2 d'acceleration : suffisant
    # pour des mouvements de quelques degres en quelques secondes sans rebond.
    for joint in ["joint_1", "joint_2", "joint_3"]:
        limits[joint] = {
            "has_velocity_limits": True,
            "max_velocity": 0.2,
            "has_acceleration_limits": True,
            "max_acceleration": 0.5,
        }
    for joint in ["joint_4", "joint_5"]:
        limits[joint] = {
            "has_velocity_limits": True,
            "max_velocity": 2.0,
            "has_acceleration_limits": True,
            "max_acceleration": 2.0,
        }

    # Launch move_group (delayed to let controllers start first)
    move_group_actions = generate_move_group_launch(moveit_config).entities

    move_group_delayed = TimerAction(
        period=8.0,
        actions=list(move_group_actions),
    )

    return LaunchDescription(
        [
            rsp_node,
            control_node,
            static_tf,
            jsb_spawner,
            arm_spawner,
            move_group_delayed,
        ]
    )
