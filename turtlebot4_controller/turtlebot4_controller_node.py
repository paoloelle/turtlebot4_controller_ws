

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
#import turtlebot4_controller.ANN_controller as controller
from math import pi

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


        self.filtered_scan = []
        
        self.section_limits = [(-pi/8, pi/8), (pi/8, 3/8*pi), (3/8*pi, 5/8*pi),
                               (5/8*pi, 7/8*pi), (-pi/8, -3/8*pi), (-3/8*pi, -5/8*pi),
                               (-5/8*pi, -7/8*pi), (-7/8*pi, 7/8*pi)]

        #self.ann_controller = controller(input_size, hidden_size, output_size)

        
        
    
    def hazard_callback(self, msg): # care only about bumper collision
        print(' ')



    def scan_callback(self, scan_msg):

        # split laser scan lectures in 8 sections as follows:
        # [-pi/8, pi/8], [pi/8, 3/8 pi], [3/8 pi, 5/8 pi], [5/8 pi, 7/8 pi]
        # [-pi/8, -3/8 pi], [-3/8 pi, -5/8 pi], [-5/8 pi, -7/8 pi], [-7/8 pi, 7/8 pi]

        switch_lectures = True  
        
        for angle_limits in self.section_limits:
            min_angle_lim, max_angle_lim = angle_limits

            min_index, max_index = self.get_reference_index(scan_msg, min_angle_lim, max_angle_lim)  

            if angle_limits[-1][0] == min_angle_lim and angle_limits[-1][1] == max_angle_lim:
                switch_lectures = True

            min_distance = self.get_min_distance(scan_msg, min_index, max_index, switch_lectures)
            
            self.filtered_scan.append(min_distance)
            
        print(self.fiiltred_scan)
        
        


        
    def get_min_distance(self, scan_msg, min_index, max_index, switch_lectures):

        

        if switch_lectures:
            section_lectures = [scan_msg.ranges[0:min_index], scan_msg.ranges[max_index, 0]]
        else: 
            section_lectures = scan_msg.ranges[min_index:max_index]


        # normalise lectures between 0 and 1
        normalized_lectures = [(x - scan_msg.range_min) / (scan_msg.range_max - scan_msg.range_min) for x in section_lectures]
        normalized_lectures = [scan_msg.range_min if x==float('inf') else x for x in normalized_lectures] # remove inf values

        return min(normalized_lectures)

        
        


    @staticmethod
    def get_reference_index(scan_msg, min_angle, max_angle):
                  
        min_angle = min_angle + pi
        max_angle = max_angle + pi

        msg_angle_min = scan_msg.angle_min + pi

        # compute correspondig index
        lower_index = int((min_angle - msg_angle_min) /scan_msg.angle_increment)
        upper_index = int((max_angle - msg_angle_min) /scan_msg.angle_increment)

        if lower_index > upper_index:
            lower_index, upper_index = upper_index, lower_index

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