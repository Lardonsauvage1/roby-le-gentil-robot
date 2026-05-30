"""Launch ros2_control with 3 real steppers (axes 1-3), joints 4-5 mock.

Run on Pi5:
    export ROS_DOMAIN_ID=42
    ros2 launch roby_hardware steppers_test.launch.py

The PC runs MoveIt separately via:
    ros2 launch neuroneimitationcarote_moveit_config moveit_only.launch.py
"""

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

    # Robot state publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_param}],
    )

    # ros2_control_node (real hardware)
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

    # Spawn controllers after 3s delay
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

    # Static TF: world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="both",
    )

    return LaunchDescription(
        [
            rsp_node,
            control_node,
            static_tf,
            jsb_spawner,
            arm_spawner,
        ]
    )
