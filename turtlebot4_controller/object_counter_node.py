import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose

class Object_Counter(Node):

    

    def __init__(self):
        
    
        super().__init__('object_counter')

        # subscriber
        self.scan_subscription = self.create_subscription(
                Pose,
                '/model/objects/pose',
                self.object_callback,
                qos_profile_sensor_data
            )
    
    def object_callback(self, pose_msg):
        if pose_msg.position.y < 0.5:
            Object_Counter.object_nest[i] = True
        else:
            Object_Counter.object_nest[i] = False
        
        self.get_logger().info('Object Nest: "%s"' % sum(Object_Counter.object_nest))



def main(args=None):
    
    rclpy.init(args=args)
    object_counter_node = Object_Counter()
    rclpy.spin(object_counter_node)
    object_counter_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()