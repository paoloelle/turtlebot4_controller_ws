import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16

class Cliff_Sensor(Node):

    # limits of the 4 different areas
    nest_lim = 0.5
    cache_lim = 1.5
    slope_lim = 2.6

    def __init__(self):

        super().__init__('cliff_sensor_node')

        #subscriber
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data            
        )

        # publisher
        self.intensity_publisher = self.create_publisher(UInt16, 'cliff_intensity', qos_profile_sensor_data)

    def odom_callback(self, odom_msg):
        
        y_pose = odom_msg.pose.pose.position.y

        intensity_msg = UInt16()

        if y_pose <= Cliff_Sensor.nest_lim:
            intensity_msg.data = 1000
        elif y_pose <= Cliff_Sensor.cache_lim:
            intensity_msg.data = 2000
        elif y_pose <= Cliff_Sensor.slope_lim:
            intensity_msg.data = 3000
        else:
            intensity_msg.data = 4000

        self.intensity_publisher.publish(intensity_msg)


def main(args=None):
    rclpy.init(args=args)
    cliff_sensor_node = Cliff_Sensor()
    rclpy.spin(cliff_sensor_node)
    cliff_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()