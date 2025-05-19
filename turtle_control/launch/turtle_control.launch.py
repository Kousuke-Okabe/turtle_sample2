from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('turtle_control'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='turtle_control',
            executable='turtle_control',
            name='turtle_control',
            output='screen',
            parameters=[param]
        )
    ])