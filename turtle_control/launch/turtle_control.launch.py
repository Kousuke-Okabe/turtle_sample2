from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('turtle_control'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        
        Node(
            package='turtle_control',
            executable='turtle_control',
            name='turtle_control',
            parameters=[config]
        )
    ])