"""move_group SEUL, vitesse libre x4 (vel 0.4 / accel 0.8 j1-3).
A lancer pendant que ros2_control tourne deja (pas de re-home)."""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    roby_hw_share = FindPackageShare("roby_hardware")
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([roby_hw_share, "config", "roby_motor1_test.urdf.xacro"]),
    ])
    rd = ParameterValue(robot_description, value_type=str)

    mc = MoveItConfigsBuilder(
        "neuroneimitationcarote",
        package_name="neuroneimitationcarote_moveit_config").to_moveit_configs()
    mc.robot_description = {"robot_description": rd}
    lim = mc.joint_limits["robot_description_planning"]["joint_limits"]
    for j in ["joint_1", "joint_2", "joint_3"]:
        lim[j] = {"has_velocity_limits": True, "max_velocity": 3.2,
                  "has_acceleration_limits": True, "max_acceleration": 0.8}
    for j in ["joint_4", "joint_5"]:
        lim[j] = {"has_velocity_limits": True, "max_velocity": 2.0,
                  "has_acceleration_limits": True, "max_acceleration": 2.0}
    mc.trajectory_execution["trajectory_execution"] = {"allowed_start_tolerance": 0.0}
    return LaunchDescription(list(generate_move_group_launch(mc).entities))
