import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt16
from geometry_msgs.msg import PoseArray

# this node emulate a cliff sensor, two basic solution can be implemeted:
# 1- read the position of the robot and output a reading that can be seen like the average reading of the four sensors
# 2- create the tf static (one for each cliff sensor) like for the light sensors and based on the position of each sensor output a value 

class Cliff_Sensor(Node):

    # limits of the 4 different areas
    nest_lim = 0.5
    cache_lim = 1.5
    slope_lim = 2.6

    def __init__(self):

        super().__init__('cliff_sensor_node')

        #subscriber
        self.odom_subscription = self.create_subscription(
            PoseArray,
            '/model/turtlebot4/pose',  
            self.pose_callback,
            qos_profile_sensor_data            
        )

        # publisher
        self.intensity_publisher = self.create_publisher(UInt16, 'cliff_intensity', qos_profile_sensor_data)

    def pose_callback(self, pose_msg):
        
        y_pose = pose_msg.poses[0].position.y

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