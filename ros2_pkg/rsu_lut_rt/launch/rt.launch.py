from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare("rsu_lut_rt"), "config", "rt.yaml"])
    config = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="rsu_lut_rt",
                executable="state_node",
                name="rsu_lut_state_node",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rsu_lut_rt",
                executable="solution_node",
                name="rsu_lut_solution_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
