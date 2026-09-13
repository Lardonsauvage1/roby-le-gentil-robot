"""Stack minimale sans materiel : ros2_control (RobySystem, joint_5 bldc) + noeud wrist_bldc simule.

    ROS_DOMAIN_ID=88 ros2 launch <ce fichier>          (domaine isole, jamais 42)
"""

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
NEST = "0.2009"


def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                os.path.join(HERE, "roby_bldc_bridge_sim.urdf.xacro"),
                " nest:=" + NEST,
            ]
        ),
        value_type=str,
    )
    controllers = PathJoinSubstitution([FindPackageShare("roby_hardware"), "config", "roby_controllers.yaml"])

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
            output="both",
        )

    return LaunchDescription(
        [
            Node(
                package="roby_wrist_bldc",
                executable="wrist_bldc_node",
                name="wrist_bldc",
                namespace="roby",
                parameters=[{"simulate": True, "nest_position": float(NEST)}],
                output="both",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="both",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"robot_description": robot_description}, controllers],
                output="both",
            ),
            TimerAction(period=3.0, actions=[spawner("joint_state_broadcaster"), spawner("arm_controller")]),
        ]
    )
