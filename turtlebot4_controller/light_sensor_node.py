import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import Odometry

class Light_Sensor(Node):

    lightPoses = None

    def __init__(self):

        super()__init__('light_sensor_node')

        self.light_pose_subscription = self.create_subscription(
            PoseArray,
            'model/lights/pose',
            self.light_pose_callback
            qos_profile_sensor_data
        )

        self.robot_position_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

    def light_pose_callback(self, light_pose_msg):

        if lightPoses:        
            self.light_pose_subscription.shutdown()
        else:
            for light_pose in light_pose_msg.pose:
                Light_Sensor.lightPoses.append(light_pose.position)





def main(args=None):

    rclpy.init(args=args)
    light_sensor_node = Light_Sensor()
    light_sensor_node.spin()
    light_sensor_node.destroy_node()
    light_sensor_node.shutdown()

if name == '__main__':
    main()