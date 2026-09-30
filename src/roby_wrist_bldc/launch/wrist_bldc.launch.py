"""Noeud du poignet BLDC seul (banc, ou inclus par robot_control.launch.py wrist:=bldc).

    ros2 launch roby_wrist_bldc wrist_bldc.launch.py                  # carte reelle
    ros2 launch roby_wrist_bldc wrist_bldc.launch.py simulate:=true   # sans materiel

nest_position : par defaut joint_5 de roby_hardware/config/initial_positions.yaml.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def default_nest_position() -> float:
    path = os.path.join(
        get_package_share_directory("roby_hardware"), "config", "initial_positions.yaml"
    )
    with open(path, encoding="utf-8") as fh:
        return float(yaml.safe_load(fh)["initial_positions"]["joint_5"])


def _setup(context):
    params_file = os.path.join(
        get_package_share_directory("roby_wrist_bldc"), "config", "wrist_bldc.yaml"
    )
    nest = LaunchConfiguration("nest_position").perform(context)
    overrides = {
        "simulate": LaunchConfiguration("simulate").perform(context) == "true",
        "port": LaunchConfiguration("port").perform(context),
        "nest_position": float(nest) if nest else default_nest_position(),
    }
    return [
        Node(
            package="roby_wrist_bldc",
            executable="wrist_bldc_node",
            name="wrist_bldc",
            namespace="roby",
            parameters=[params_file, overrides],
            output="both",
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("simulate", default_value="false"),
            DeclareLaunchArgument("port", default_value="auto"),
            DeclareLaunchArgument(
                "nest_position",
                default_value="",
                description="vide = initial_positions.yaml",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
