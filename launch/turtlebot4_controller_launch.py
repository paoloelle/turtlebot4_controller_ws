import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

# TODO
# launch file for the gazebo simulation
# launch the node to control the turtlebot with the weights from the neuroevolution
# save the number of object stored 

def generate_launch_description():
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package = 'turtlebot4_controller',
            executable='turtlebot4_controller_node',
        ),

        DeclareLaunchArgument(
            'world',
            default_value='arena'
        ),

        DeclareLaunchArgument(
            'z',
            default_value='0.2'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("turtlebot4_ignition_bringup"), '/launch', '/turtlebot4_ignition.launch.py'
            ]),

        launch_arguments={
            'world':LaunchConfiguration('world'),
            'z':LaunchConfiguration('z')
        }.items()

        )

        
        
    ])