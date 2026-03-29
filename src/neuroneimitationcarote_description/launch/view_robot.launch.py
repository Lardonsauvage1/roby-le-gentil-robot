from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    urdf_path = PathJoinSubstitution([
        FindPackageShare("neuroneimitationcarote_description"),
        "urdf", "robot.urdf.xacro"
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_path]),
            value_type=str
        )
    }

    return LaunchDescription([

        # Publie le URDF sur /robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
        ),

        # GUI pour bouger les joints manuellement (test sans hardware)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),

        # RViz pour visualiser
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("neuroneimitationcarote_description"),
                "launch", "view_robot.rviz"
            ])],
        ),
    ])
