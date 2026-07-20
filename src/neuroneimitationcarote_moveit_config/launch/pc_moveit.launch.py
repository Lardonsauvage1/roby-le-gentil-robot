"""MoveIt (move_group) + RViz sur le PC — rééquilibrage archi 2026-06-26.

Le Pi5 lance robot_control.launch.py (ros2_control + rsp + spawners, temps-réel).
Le PC lance ce fichier : move_group planifie et pilote l'arm_controller du Pi5
via l'action FollowJointTrajectory (DDS), RViz affiche l'état réel.

IMPORTANT — un SEUL publisher de /robot_description (le rsp du Pi5, URDF hardware
réel). Ici on NE lance PAS de robot_state_publisher (sinon il publierait l'URDF
mock/FakeSystem du moveit_config et le ros2_control_node du Pi pourrait charger le
mock — cf rviz_only.launch.py / robot_full.launch.py). move_group reçoit
robot_description en paramètre (même cinématique) pour planifier, c'est suffisant.
Le static_tf world→base_link est aussi fourni par le Pi5.

Usage (PC) :
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///home/sam/cyclone_config.xml
    unset GTK_PATH
    ros2 launch neuroneimitationcarote_moveit_config pc_moveit.launch.py
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config"
    ).to_moveit_configs()

    # Force move_group à utiliser la MÊME description que le control_node du Pi5
    # n'est pas possible ici (URDF distant) ; on garde l'URDF du moveit_config
    # pour la PLANIFICATION (cinématique identique). L'exécution se fait sur le Pi.

    # Limites conservatrices (steppers lents, sécurité). À remonter plus tard une
    # fois le bit-bang/overrun traité (cf project_gpio_overrun_analyse).
    limits = moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
    for joint in ["joint_1", "joint_2", "joint_3"]:
        limits[joint] = {
            "has_velocity_limits": True,
            "max_velocity": 0.4,
            "has_acceleration_limits": True,
            "max_acceleration": 0.8,
        }
    for joint in ["joint_4", "joint_5"]:
        limits[joint] = {
            "has_velocity_limits": True,
            "max_velocity": 2.0,
            "has_acceleration_limits": True,
            "max_acceleration": 2.0,
        }

    # Open-loop : on désactive le contrôle de tolérance start-state (steppers sans
    # encodeur actif) — sinon ABORT "start point deviates" après replanification.
    moveit_config.trajectory_execution["trajectory_execution"] = {
        "allowed_start_tolerance": 0.0,
    }

    ld = LaunchDescription()

    # move_group (planification + interface controllers du Pi5)
    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)

    # RViz avec le plugin MoveIt (visualisation + cible interactive)
    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(action)

    # PAS de robot_state_publisher (le Pi5 est l'unique publisher /robot_description).
    # PAS de static_tf world→base_link (fourni par le Pi5).

    return ld
