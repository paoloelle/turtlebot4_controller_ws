from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.09', '--y', '0.08', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'light_sensor_frontL']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.09', '--y', '-0.08', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'light_sensor_frontR']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '-0.11', '--y', '0', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'light_sensor_back']
        ),
    ])