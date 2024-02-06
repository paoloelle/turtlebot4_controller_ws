import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# TODO
# launch file for the gazebo simulation
# launch the node to control the turtlebot
# save the number of object stored 

def generate_launch_description():
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package = 'turtlebot4_controller',
            executable='turtlebot4_controller_node',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("turtlebot4_ignition_bringup"), '/launch', '/turtlebot4_ignition.launch.py'
            ])      

        )

        
        
    ])