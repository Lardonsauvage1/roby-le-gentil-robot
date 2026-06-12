"""
Lance UNIQUEMENT le noeud MoveIt Servo pour Roby.

À utiliser EN PLUS de demo.launch.py (qui fournit le robot simulé mock,
les contrôleurs, move_group et RViz).

Étapes de test (étape 1) :
  Terminal A : ros2 launch neuroneimitationcarote_moveit_config demo.launch.py
  Terminal B : ros2 launch roby_control servo_sim.launch.py
  Puis      : ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
              ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped ...
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Config MoveIt du robot (URDF + SRDF + cinématique + limites).
    moveit_config = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config",
    ).to_moveit_configs()

    # Paramètres Servo, chargés sous le namespace "moveit_servo".
    servo_params = {
        "moveit_servo": ParameterBuilder("roby_control")
        .yaml("config/roby_servo.yaml")
        .to_dict()
    }

    # Le filtre de lissage (AccelerationLimitedPlugin) a besoin de connaître
    # le groupe et sa période de mise à jour.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
