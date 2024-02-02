

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
#import turtlebot4_controller.ANN_controller as controller
from math import pi, copysign

class Controller_Node(Node):

    def __init__(self, input_size, hidden_size, output_size):

        super().__init__('ann_controller')

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.hazard_subscription= self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.hazard_callback,
            qos_profile_sensor_data
        )

        self.bumper = 0

        self.filtered_scan = None 

        #self.ann_controller = controller(input_size, hidden_size, output_size)

        
        


    
    def hazard_callback(self, msg): # care only about bumper collision
        print(' ')



    def scan_callback(self, scan_msg):
        
        self.get_min_distance(scan_msg)



    #lin_vel, ang_vel = self.ann.forward(sensors_data) qui ottengo i valori da pubblicare nel topic cmd_vel
        
    def get_min_distance(self, scan_msg):

        min_index, max_index = self.get_reference_index(scan_msg, pi/8, pi)
        print(min_index)
        print(max_index)

        
        

        #TODO normalize ranges between 0 and 1

    def get_reference_index(scan_msg, min_angle, max_angle):
                  
        min_angle = min_angle + pi
        max_angle = max_angle + pi

        msg_angle_min = scan_msg.angle_min + pi

        # compute correspondig index
        lower_index = int((min_angle - msg_angle_min) /scan_msg.angle_increment)
        upper_index = int((max_angle - msg_angle_min) /scan_msg.angle_increment)

        lower_index = 1
        upper_index = 2

        return lower_index, upper_index







def main(args=None):

    rclpy.init(args=args)

    controller_node = Controller_Node(202, 8, 2)
    #weights = controller_node.get_parameter('weights').get_parameter_value(). #TODO

    #controller_node.ann_controller.upload_parameters(weights)# parameters from neuroevolution
    rclpy.spin(controller_node)
    
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()