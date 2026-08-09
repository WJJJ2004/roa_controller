from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rsu_manager")
    solver_param_file = PathJoinSubstitution([pkg, "config", "rsu_imp_sample.yaml"])
    sample_param_file = PathJoinSubstitution([pkg, "config", "pace_sample_cfg.yaml"])
    return LaunchDescription([
        Node(
            package="rsu_manager",
            executable="rt_imp_solver_node",
            name="rt_imp_solver_node",
            output="screen",
            parameters=[
                solver_param_file,
            ],
            arguments=["--ros-args", "--log-level", "info"],
        ),
        Node(
                package="rsu_manager",
                executable="pace_symmetric_sample_generator_node",
                name="pace_symmetric_sample_generator_node",
                output="screen",
                parameters=[
                    sample_param_file,
                ],
                arguments=["--ros-args", "--log-level", "warn"],
        ),
    ])