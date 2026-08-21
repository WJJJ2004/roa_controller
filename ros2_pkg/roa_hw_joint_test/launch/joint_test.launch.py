from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("roa_hw_joint_test")
    visualizer_share = FindPackageShare("roa_visualizer")

    amplitude_deg = LaunchConfiguration("amplitude_deg")
    visualize = LaunchConfiguration("visualize")

    return LaunchDescription([
        DeclareLaunchArgument("amplitude_deg", default_value="30.0"),
        DeclareLaunchArgument("visualize", default_value="true"),
        Node(
            package="roa_hw_joint_test",
            executable="mirrored_joint_test_node",
            name="mirrored_joint_test_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    package_share, "config", "joint_test.yaml"
                ]),
                {
                    "amplitude_deg": amplitude_deg,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                visualizer_share, "launch", "display.launch.py"
            ])),
            condition=IfCondition(visualize),
        ),
    ])
