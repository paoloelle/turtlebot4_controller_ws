import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from std_msgs.msg import Float32

class LightSensor(Node):

    def __init__(self):

        self.light_sensors_list = ['light_sensor_frontL', 'light_sensor_frontR', 'light_sensor_back']
        self.base_frame = 'arena' # to get the position related to the arena
        
        super().__init__('light_sensor_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

        # create publishers
        self.light_publishers = {}
        for light in self.light_sensors_list:
            self.light_publishers[light] = self.create_publisher(Float32, light, 1)


    def on_timer(self):

        for light in self.light_sensors_list:

            target_frame = light

            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame,
                    self.base_frame,
                    rclpy.time.Time())


            except TransformException as ex:
                self.get_logger().info(f'Could not transform {target_frame} to {self.base_frame}: {ex}')
                return

            # create and publish light message
            light_sensor_msg = Float32()
            light_sensor_msg.data = 1 / (abs(t.transform.translation.x) + abs(t.transform.translation.y) + abs(t.transform.translation.z))
            self.light_publishers[light].publish(light_sensor_msg)



def main(args=None):
    rclpy.init(args=args)
    light_sensor_node = LightSensor()
    rclpy.spin(light_sensor_node)
    light_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

