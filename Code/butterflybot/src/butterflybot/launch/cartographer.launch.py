from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('butterflybot'),
        'config'
    )

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'butterflybot_2d.lua'
            ],
            remappings=[
                ('scan', '/lidar_scan')
            ]
        )
    ])

