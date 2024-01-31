

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
#import turtlebot4_controller.ANN_controller as controller

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

        
        self.declare_parameter('weights', [1]*(input_size*hidden_size)+(hidden_size*output_size)) 
        
        #self.ann_controller = controller(input_size, hidden_size, output_size)

        self.bumper = 0

        self.scan_lectures = None
        


    
    def hazard_callback(self, msg): # care only about bumper collision
        if len(msg.detections) != 0:
            hazard = msg.detections[0]
            if hazard.type == 2:
                self.get_logger().info('Collision')
                self.bumper = 1
        else:
            self.bumper = 0



    def scan_callback(self, msg):
        
        min_index = int((self.min_angle - msg.angle_min) / msg.angle_increment)
        max_index = int((self.max_angle - msg.angle_min) / msg.angle_increment)

        min_index = max(0, min(min_index, len(msg.ranges) - 1))
        max_index = max(0, min(max_index, len(msg.ranges) - 1))


        filtered_ranges = msg.ranges[min_index:max_index-1]

        

        normalized_ranges = [(x - msg.range_min) / (msg.range_max - msg.range_min) * (msg.range_max - msg.range_min) + msg.range_min for x in filtered_ranges]
        normalized_ranges = [msg.range_min if x==float('inf') else x for x in normalized_ranges] # remove inf values

        #filtered_scan.ranges = normalized_ranges

        #self.publisher.publish(filtered_scan)
        print(len(normalized_ranges))


    #lin_vel, ang_vel = self.ann.forward(sensors_data) qui ottengo i valori da pubblicare nel topic cmd_vel

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