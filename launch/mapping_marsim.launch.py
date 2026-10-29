from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'config', 'marsim.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
             output='screen', parameters=[LaunchConfiguration('config'),
                                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
