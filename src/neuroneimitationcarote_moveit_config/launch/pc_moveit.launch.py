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

Axe 5 : wrist:=bldc (defaut depuis le 2026-09-30, meme defaut que robot_control.launch.py
sur le Pi5) limite joint_5 a ce que le poignet BLDC suit reellement (0.5 rad/s).
wrist:=servo : ancien servo provisoire, demonte.

Scene de collision : chargee ICI, automatiquement (scene:=cuisine par defaut, scene:=aucune
pour s'en passer). Elle vit dans move_group : chaque relance la perdait, et on l'a oubliee
pendant toute une seance d'essais du modele (2026-09-13). rviz:=false pour les essais sans ecran.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrist",
                default_value="bldc",
                choices=["servo", "bldc"],
                description="actionneur de l'axe 5 : bldc (poignet monte, defaut) ou servo (ancien servo provisoire, demonte)",
            ),
            DeclareLaunchArgument(
                "scene",
                default_value="cuisine",
                description="scene roby_environments chargee au demarrage (aucune = pas de scene)",
            ),
            DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
            OpaqueFunction(function=_setup),
        ]
    )


def _setup(context):
    wrist = LaunchConfiguration("wrist").perform(context)
    scene = LaunchConfiguration("scene").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
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
    if wrist == "bldc":
        # Carte : 15 rad/s moteur = 1.67 rad/s bras (reducteur planetaire 9:1) ; le noeud
        # wrist_bldc rampe a 0.5 rad/s / 1.5 rad/s2. Planifier plus vite ferait
        # trainer l'axe derriere la trajectoire.
        limits["joint_5"] = {
            "has_velocity_limits": True,
            "max_velocity": 0.5,
            "has_acceleration_limits": True,
            "max_acceleration": 1.5,
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
    if rviz == "true":
        for action in generate_moveit_rviz_launch(moveit_config).entities:
            ld.add_action(action)

    # Scene de collision : le chargeur attend move_group, charge, et s'arrete.
    if scene != "aucune":
        ld.add_action(
            Node(
                package="roby_environments",
                executable="scene_loader",
                arguments=["--env", scene, "--attente", "90"],
                output="both",
            )
        )

    # PAS de robot_state_publisher (le Pi5 est l'unique publisher /robot_description).
    # PAS de static_tf world→base_link (fourni par le Pi5).

    return [ld]
