"""Existing walking controller + v3; hardware startup remains an explicit choice."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rsu_manager_v3')
    limit = ParameterValue(LaunchConfiguration('motor_torque_limit'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument('motor_torque_limit', default_value='11.9'),
        DeclareLaunchArgument('hardware_config', default_value=PathJoinSubstitution([
            FindPackageShare('robstride_hardware_interface'), 'config', 'motor_setting.yaml'])),
        Node(package='rsu_manager_v3', executable='rsu_manager_v3_node',
             name='rsu_manager_v3', output='screen', parameters=[
                 PathJoinSubstitution([share, 'config', 'rsu_manager_v3.yaml']),
                 {'motor_torque_limit': limit}]),
        Node(package='roa_main_controller', executable='roa_main_controller',
             name='roa_main_controller', output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('roa_main_controller'), 'config', 'params.yaml']),
                 {'rsu_control_mode': 'torque', 'REALTIME_CONTROL_MODE': True}]),
        Node(package='robstride_hardware_interface', executable='hardware_interface_node',
             name='hardware_interface_node', output='screen',
             condition=IfCondition(LaunchConfiguration('start_hardware')), parameters=[
                 LaunchConfiguration('hardware_config'),
                 {'rsu_control_mode': 'torque', 'control_frequency_hz': 200.0,
                  'rsu_motor_torque_limit': limit}]),
    ])
