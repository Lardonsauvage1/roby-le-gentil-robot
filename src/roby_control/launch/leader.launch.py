"""Lance leader_node (bras guide SO-ARM 101).

    ros2 launch roby_control leader.launch.py
    ros2 launch roby_control leader.launch.py simulate:=true
    ros2 launch roby_control leader.launch.py ids:="[1,2,3,4,5]"

⚠️ A lancer via ~/roby_leader_node.sh : le noeud a besoin d'un interpreteur ou ROS 2
ET scservo_sdk coexistent, ce qui n'est pas le cas du python systeme.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("simulate", default_value="false",
                              description="true = aucun materiel requis"),
        DeclareLaunchArgument("port", default_value="/dev/roby_leader"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("telemetry_rate_hz", default_value="1.0"),
    ]
    node = Node(
        package="roby_control",
        executable="leader_node",
        name="leader_node",
        output="both",
        emulate_tty=True,
        parameters=[{
            "simulate": LaunchConfiguration("simulate"),
            "port": LaunchConfiguration("port"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "telemetry_rate_hz": LaunchConfiguration("telemetry_rate_hz"),
        }],
    )
    return LaunchDescription(args + [node])
