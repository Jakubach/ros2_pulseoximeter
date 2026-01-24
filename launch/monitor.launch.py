from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('ros2_pulseoximeter'),
        'config',
        'monitor_config.yaml'
    ])

    return LaunchDescription([
        Node(
            package='ros2_pulseoximeter',
            executable='monitor_node',
            name='monitor_node',
            parameters=[{'config_path': config_path}],
            output='screen'
        )
    ])
