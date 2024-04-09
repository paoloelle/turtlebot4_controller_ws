from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # nodes lights tf

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.09', '--y', '0.08', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'turtlebot4', '--child-frame-id', 'light_sensor_frontL']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.09', '--y', '-0.08', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'turtlebot4', '--child-frame-id', 'light_sensor_frontR']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '-0.11', '--y', '0', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'turtlebot4', '--child-frame-id', 'light_sensor_back']
        ),

        # node to remap tf from map -> robot pose

        #Node(
        #    package= 'ros_ign_bridge',
        #    executable= 'parameter_bridge',
        #    arguments= '/model/turtlebot4/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        #    remappings= (['/model/turtlebot4/pose'], '/tf')
        #)

    ])

