"""Faux robot COMPLET pour valider la teleop du vrai bras sans le vrai bras.

Materiel simule (mock_components) + arm_controller (JTC) regle comme sur le Pi
(open_loop_control) + move_group + TF. Le garde et le noeud de teleop se lancent a
cote, exactement comme sur la vraie stack.

    ROS_DOMAIN_ID=43 ros2 launch <ce fichier> [rviz:=true]

Refuse de demarrer sur le domaine 42 (celui de la vraie stack).
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)

ICI = Path(__file__).resolve().parent


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID", "0") == "42":
        raise RuntimeError("domaine 42 = vraie stack : lancer ce faux robot sur un autre "
                           "domaine (ROS_DOMAIN_ID=43)")
    mc = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config").to_moveit_configs()
    # Memes limites que demo.launch.py et pc_moveit.launch.py.
    limits = mc.joint_limits["robot_description_planning"]["joint_limits"]
    for j, v in (("joint_1", 1.0), ("joint_2", 1.0), ("joint_3", 1.0),
                 ("joint_4", 2.0), ("joint_5", 2.0)):
        limits[j] = {"has_velocity_limits": True, "max_velocity": v,
                     "has_acceleration_limits": True, "max_acceleration": v}

    ld = LaunchDescription([DeclareLaunchArgument("rviz", default_value="false")])
    for a in generate_rsp_launch(mc).entities:
        ld.add_action(a)
    cm = Node(
        package="controller_manager", executable="ros2_control_node", output="screen",
        parameters=[mc.robot_description,
                    str(mc.package_path / "config/ros2_controllers.yaml"),
                    str(ICI / "open_loop.yaml")])
    ld.add_action(cm)
    spawn = [Node(package="controller_manager", executable="spawner", output="screen",
                  arguments=[c, "--controller-manager-timeout", "30"])
             for c in ("joint_state_broadcaster", "arm_controller")]
    ld.add_action(RegisterEventHandler(OnProcessStart(
        target_action=cm, on_start=[TimerAction(period=3.0, actions=spawn)])))
    for a in generate_move_group_launch(mc).entities:
        ld.add_action(a)
    ld.add_action(GroupAction(list(generate_moveit_rviz_launch(mc).entities),
                              condition=IfCondition(LaunchConfiguration("rviz"))))
    ld.add_action(Node(package="tf2_ros", executable="static_transform_publisher",
                       arguments=["--frame-id", "world", "--child-frame-id", "base_link"]))
    return ld
