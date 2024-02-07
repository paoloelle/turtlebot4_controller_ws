import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from turtlebot4_controller.ann_controller import ANN_controller
from math import pi
from geometry_msgs.msg import Twist
import numpy as np

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

        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile_sensor_data)


        self.filtered_scan = [1.0]*8 # initialize with a dummy lecture
        
        self.SECTION_LIMITS = [(-pi/8, pi/8), (pi/8, 3/8*pi), (3/8*pi, 5/8*pi),
                               (5/8*pi, 7/8*pi), (-pi/8, -3/8*pi), (-3/8*pi, -5/8*pi),
                               (-5/8*pi, -7/8*pi), (-7/8*pi, 7/8*pi)]
        
        self.bumper = 0 # 0 = no collision, 1 = collision

        self.ann_controller = ANN_controller(input_size, hidden_size, output_size)

        self.declare_parameter('weights', [0.0]*(input_size*hidden_size + hidden_size*output_size))

        
        
    
    def hazard_callback(self, hazard_msg): # care only about bumper collision

        if not hazard_msg.detections:
            self.bumper = 0
            
        else:
            for hazards in hazard_msg.detections:
                if hazards.type == 1:
                    self.bumper = 1
                    self.get_logger().warning('collision')
                else:
                    self.bumper = 0

        
        # neural network prediction
        lin_vel, ang_vel = self.ann_controller.forward([self.bumper] + self.filtered_scan)
        self.publish_twist(lin_vel, ang_vel)


    def scan_callback(self, scan_msg):


        # split laser scan lectures in 8 sections as follows:
        # [-pi/8, pi/8], [pi/8, 3/8 pi], [3/8 pi, 5/8 pi], [5/8 pi, 7/8 pi]
        # [-pi/8, -3/8 pi], [-3/8 pi, -5/8 pi], [-5/8 pi, -7/8 pi], [-7/8 pi, 7/8 pi]

        switch_lectures = False  
        self.filtered_scan.clear() # remove previous values

        for angle_limits in self.SECTION_LIMITS:
            min_angle_lim, max_angle_lim = angle_limits

            min_index, max_index = self.get_reference_index(scan_msg, min_angle_lim, max_angle_lim)  

            if self.SECTION_LIMITS[-1] == (min_angle_lim, max_angle_lim):
                switch_lectures = True

            min_distance = self.get_min_distance(scan_msg, min_index, max_index, switch_lectures)

            self.filtered_scan.append(min_distance)

        #print(self.filtered_scan)

        # neural network prediction
        lin_vel, ang_vel = self.ann_controller.forward([self.bumper] + self.filtered_scan)
        self.publish_twist(lin_vel, ang_vel)


    
    def publish_twist(self, lin_vel, ang_vel):

        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel

        self.twist_publisher.publish(twist_msg)
        self.get_logger().info('\nLinear vel: "%s" \nAngular vel:  "%s"' % (lin_vel, ang_vel))

            
        
    @staticmethod
    def get_min_distance(scan_msg, min_index, max_index, switch_lectures):

        if switch_lectures:
            section_lectures = scan_msg.ranges[0:min_index] + scan_msg.ranges[max_index:]
        else: 
            section_lectures = scan_msg.ranges[min_index:max_index]


        # normalise lectures between 0 and 1
        normalized_lectures = [(x - scan_msg.range_min) / (scan_msg.range_max - scan_msg.range_min) for x in section_lectures]
        normalized_lectures = [scan_msg.range_max if x==float('inf') else x for x in normalized_lectures] # remove inf values
        

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

    INPUT_SIZE = 9 # BUMPER (1) + LASER SCAN FILTERED (8)
    HIDDEN_SIZE = 8
    OUTPUT_SIZE = 2 # linear vel x, angular vel z

    controller_node = Controller_Node(INPUT_SIZE, HIDDEN_SIZE, OUTPUT_SIZE)
    
    
    weights = controller_node.get_parameter('weights').get_parameter_value().double_array_value

    controller_node.ann_controller.upload_parameters(weights)

    rclpy.spin(controller_node)
    
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()