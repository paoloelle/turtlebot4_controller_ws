import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('turtlebot4_controller'),
        'config',
        'weights.yaml'
    )

    return LaunchDescription([
        Node(
            package='turtlebot4_controller',
            executable='turtlebot4_controller_node',
            name='ANN_controller',
            parameters=[config]            
        )
    ])