"""Launch ros2_control with motor 1 real, joints 2-5 mock.

Run on Pi5:
  ros2 launch roby_hardware motor1_test.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    roby_hw_share = FindPackageShare("roby_hardware")

    # Generate URDF with motor1-only hardware config
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

    # Robot state publisher (publishes URDF to robot_description topic)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_param}],
    )

    # ros2_control_node
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
            rsp_node,
            control_node,
            static_tf,
            jsb_spawner,
            arm_spawner,
        ]
    )
