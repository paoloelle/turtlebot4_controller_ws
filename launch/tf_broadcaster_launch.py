from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # nodes to map static tf from turtlebot4 -> light_sensor

        # light sensor front left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.09', '--y', '0.08', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'turtlebot4', '--child-frame-id', 'light_sensor_frontL']
        ),

        # light sensor front right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.09', '--y', '-0.08', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'turtlebot4', '--child-frame-id', 'light_sensor_frontR']
        ),

        # light sensor back (center)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '-0.11', '--y', '0', '--z', '0.35', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'turtlebot4', '--child-frame-id', 'light_sensor_back']
        ),

        # node to remap tf from map -> robot pose, used for light sensors
        Node(
            package= 'ros_ign_bridge',
            name = 'robot_pose_tf_bridge',
            executable= 'parameter_bridge',
            arguments= ['/model/turtlebot4/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
            remappings= [('/model/turtlebot4/pose', '/tf')]
        ),

        # node to remap pose from map -> robot pose, used for cliff sensors
        Node(
            package= 'ros_ign_bridge',
            name= 'robot_pose_poseArray_bridge',
            executable= 'parameter_bridge',
            arguments= ['/model/turtlebot4/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'],
        ),

    ])

