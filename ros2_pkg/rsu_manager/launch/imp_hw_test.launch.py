from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rsu_manager")
    param_file = PathJoinSubstitution([pkg, "config", "rsu_hw_test.yaml"])

    return LaunchDescription([
        Node(
            package="rsu_manager",
            executable="rt_imp_solver_node",
            name="rt_imp_solver_node",
            output="screen",
            parameters=[
                param_file,
            ],
            arguments=["--ros-args", "--log-level", "info"],
        ),
        Node(
                package="rsu_manager",
                executable="imp_hw_controll_test_node",
                name="imp_hw_controll_test_node",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
        ),
    ])