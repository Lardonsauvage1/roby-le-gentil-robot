"""Lean control launch (rebalance archi 2026-06-26) : ros2_control + RT seul.

Run on Pi5 :
    export ROS_DOMAIN_ID=42
    ros2 launch roby_hardware robot_control.launch.py

move_group (MoveIt) tourne maintenant sur le PC (pc_moveit.launch.py).
Le Pi5 ne garde que le temps-reel : rsp + ros2_control_node + spawners.
Le rsp du Pi5 est l UNIQUE publisher de /robot_description (URDF hardware reel).

Axe 5 (poignet) :
    wrist:=servo  (defaut) servo provisoire PCA9685 CH1, comportement inchange
    wrist:=bldc   nouveau poignet BLDC : lance aussi le noeud roby_wrist_bldc, qui
                  recale la carte au nid au demarrage (tete AU NID avant de lancer).
                  Cote PC : pc_moveit.launch.py wrist:=bldc (limites de vitesse).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrist",
                default_value="servo",
                choices=["servo", "bldc"],
                description="actionneur de l'axe 5 : servo (actuel) ou bldc (nouveau poignet)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )


def _setup(context):
    wrist = LaunchConfiguration("wrist").perform(context)
    roby_hw_share = FindPackageShare("roby_hardware")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [roby_hw_share, "config", "roby_motor1_test.urdf.xacro"]
            ),
            " wrist:=",
            wrist,
        ]
    )

    controllers_yaml = PathJoinSubstitution(
        [roby_hw_share, "config", "roby_controllers.yaml"]
    )

    robot_description_param = ParameterValue(robot_description, value_type=str)

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
        period=6.0,
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
        period=6.0,
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

    actions = [
        rsp_node,
        control_node,
        static_tf,
        jsb_spawner,
        arm_spawner,
    ]
    if wrist == "bldc":
        # Lance en meme temps que ros2_control : le plugin attend (8 s max) la
        # 1re mesure recalee du noeud avant d'activer joint_5.
        actions.insert(
            0,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("roby_wrist_bldc"), "launch", "wrist_bldc.launch.py"])
                )
            ),
        )
    return actions
