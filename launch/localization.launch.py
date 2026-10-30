from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'config', 'slam.yaml'])),
        DeclareLaunchArgument('localization_config', default_value=PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'config', 'localization.yaml'])),
        DeclareLaunchArgument('map_directory', description='Directory containing manifest.txt'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
             output='screen', parameters=[LaunchConfiguration('config'),
                                         LaunchConfiguration('localization_config'),
                                         {'localization.enabled': True,
                                          'localization.map_directory': ParameterValue(LaunchConfiguration('map_directory'), value_type=str),
                                          'backend.enabled': False,
                                          'pcd_save.pcd_save_en': False,
                                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
