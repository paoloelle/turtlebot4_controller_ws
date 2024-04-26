import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from math import atan2, tan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from math import pi, degrees




class Controller_Node(Node):

    def __init__(self):

        super().__init__('hand_coded_controller')

        # subscribers for the sensors

        # light sensors
        self.light_frontL_subscriber = self.create_subscription(Float32, 'light_sensor_frontL', self.light_frontL_callback, qos_profile_sensor_data) # light front left
        self.light_frontR_subscriber = self.create_subscription(Float32, 'light_sensor_frontR', self.light_frontR_callback, qos_profile_sensor_data) # light front right
        self.light_back_subscriber = self.create_subscription(Float32, 'light_sensor_back', self.light_back_callback, qos_profile_sensor_data) # light back
        
        # cliff sensors
        self.cliff_sideL_subscriber = self.create_subscription(Float32,  'cliff_sensor_side_left',   self.cliff_sideL_callback, qos_profile_sensor_data) # cliff side left
        self.cliff_sideR_subscriber = self.create_subscription(Float32,  'cliff_sensor_side_right',  self.cliff_sideR_callback, qos_profile_sensor_data) # cliff side right
        self.cliff_frontL_subscriber = self.create_subscription(Float32, 'cliff_sensor_front_left',  self.cliff_frontL_callback, qos_profile_sensor_data) # cliff front left
        self.cliff_frontR_subscriber = self.create_subscription(Float32, 'cliff_sensor_front_right', self.cliff_frontR_callback, qos_profile_sensor_data) # cliff front right
        
        # publisher
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile_sensor_data)

        # light sensors reading 
        self.light_frontL_value = None
        self.light_frontR_value = None
        self.light_back_value = None
        self.light_direction = None # light gradient direction
        
        # cliff sensors reading
        self.cliff_sideL_value  = None
        self.cliff_sideR_value  = None
        self.cliff_frontL_value = None
        self.cliff_frontR_value = None


        # FSM control variable
        self.on_source = False


        self.enable_controller = False # this variable became true when all the sensor values are not None


    # light sensors callbacks
    def light_frontL_callback(self, light_message):
        self.light_frontL_value = light_message.data
        self.compute_light_gradient()

    def light_frontR_callback(self, light_message):
        self.light_frontR_value = light_message.data
        self.compute_light_gradient()

    def light_back_callback(self, light_message):
        self.light_back_value = light_message.data
        self.compute_light_gradient()

    def compute_light_gradient(self):

        if self.light_frontL_value and self.light_frontR_value and self.light_back_value:
            dx = self.light_frontL_value - self.light_back_value 
            dy = self.light_frontL_value - self.light_frontR_value
            self.light_direction = atan2(dy, dx)

            self.control_step()
    
    
    # cliff sensors callbacks
    def cliff_sideL_callback(self, cliff_message):
        self.cliff_sideL_value = cliff_message.data
        
    def cliff_sideR_callback(self, cliff_message):
        self.cliff_sideR_value = cliff_message.data
    
    def cliff_frontL_callback(self, cliff_message):
        self.cliff_frontL_value = cliff_message.data
    
    def cliff_frontR_callback(self, cliff_message):
        self.cliff_frontR_value = cliff_message.data

    
    def control_step(self):

        self.enable_controller = all([
            #self.filtered_scan      is not None,
            self.light_direction    is not None,
            self.cliff_sideL_value  is not None,
            self.cliff_sideR_value  is not None,
            self.cliff_frontL_value is not None,
            self.cliff_frontR_value is not None
        ])
        
        twist_msg = Twist()

        if self.enable_controller:
            
            if not self.on_source: # anti phototaxis
                  lin_vel = 0.0
                  ang_vel = 0.2 * -self.light_direction

            twist_msg.linear.x = lin_vel
            twist_msg.angular.z = ang_vel

        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        
        self.twist_publisher.publish(twist_msg)

        self.get_logger().info('\nLinear vel: "%s" \nAngular vel:  "%s"' % (twist_msg.linear.x, twist_msg.angular.z))



            

    
def main(args=None):

    rclpy.init(args=args)

    controller_node = Controller_Node()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

