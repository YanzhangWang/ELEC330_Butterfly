from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='butterflybot',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen'
        )
    ])
