"""RViz viewer only — for monitoring the robot from the PC.

All computation (ros2_control, move_group, robot_state_publisher) runs on Pi5.
This launch file starts ONLY rviz2 which subscribes to /robot_description,
/joint_states and /tf via DDS.  No duplicate nodes.

Usage (PC):
    ros2 launch neuroneimitationcarote_moveit_config rviz_viewer.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("neuroneimitationcarote_moveit_config"), "config", "viewer.rviz"]
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
