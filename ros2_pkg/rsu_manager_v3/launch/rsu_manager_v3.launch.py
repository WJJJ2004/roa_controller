from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config_file")
    default = PathJoinSubstitution([FindPackageShare("rsu_manager_v3"), "config", "rsu_manager_v3.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default),
        Node(
            package="rsu_manager_v3",
            executable="rsu_manager_v3_node",
            name="rsu_manager_v3",
            output="screen",
            parameters=[config],
        ),
    ])
