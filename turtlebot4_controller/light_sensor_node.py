import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from std_msgs.msg import Float32

class LightSensor(Node):

    def __init__(self):
        
        super().__init__('light_sensor_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

        # create publisher
        self.publisher = self.create_publisher(Float32, 'light', 1)


    def on_timer(self):

        base_frame = 'odom'
        target_frame = 'light_sensor'

        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                base_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {target_frame} to {base_frame}: {ex}')

        if t:
            light_sensor_msg = Float32()
            light_sensor_msg.data = t.transform.translation.x + t.transform.translation.y + t.transform.translation.z
            
            self.publisher.publish(light_sensor_msg)


def main(args=None):
    rclpy.init(args=args)
    light_sensor_node = LightSensor()
    rclpy.spin(light_sensor_node)
    light_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

