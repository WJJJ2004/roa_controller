from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    urdf_path = Path(
        LaunchConfiguration("urdf_path").perform(context)
    )
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = urdf_path.read_text(encoding="utf-8")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="roa_visualizer",
            executable="command_joint_state_publisher",
            name="pace_command_joint_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "publish_rate_hz": 100.0,
                    "publish_before_ready": True,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("roa_visualizer")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=str(
                    package_share / "urdf" / "roa_deploy.urdf"
                ),
                description="Absolute path to the ROA URDF file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    package_share
                    / "config"
                    / "pace_visualization.rviz"
                ),
                description="Absolute path to the RViz config.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
