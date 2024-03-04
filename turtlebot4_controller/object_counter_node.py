import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray


class Object_Counter(Node):

    object_nest = [False] * 4 #FIXME number of object hardcoded

    def __init__(self):

        super().__init__('object_counter_node')

        # subscriber
        self.scan_subscription = self.create_subscription(
            PoseArray,
            '/model/objects/pose',
            self.object_callback,
            qos_profile_sensor_data
        )



    def object_callback(self, pose_msg):
        for object_index, pose_object in enumerate(pose_msg.poses[:-1]):  # drop last element cause is a dummy element (see arena.sdf for more info)
            if pose_object.position.y < 0.5:
                Object_Counter.object_nest[object_index] = True
            else:
                Object_Counter.object_nest[object_index] = False

        self.get_logger().info('Object Nest: "%s"' % sum(Object_Counter.object_nest))


def main(args=None):
    rclpy.init(args=args)
    object_counter_node = Object_Counter()
    rclpy.spin(object_counter_node)
    object_counter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
