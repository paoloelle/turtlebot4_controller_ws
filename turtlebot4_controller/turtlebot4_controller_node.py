import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from turtlebot4_controller.ann_controller import ANN_controller
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
from math import atan2, copysign
import random

class Controller_Node(Node):



    def __init__(self):

        super().__init__('ann_controller')

        #turtlebot4 max velocities
        self.MAX_ANG_VEL = 1.90 # 1.90 rad/s
        self.MAX_LIN_VEL = 0.46 # 0.46 m/s

        # threshold to determine a target velocity different from 0
        self.MIN_ANG_VEL = 0.1
        self.MIN_LIN_VEL = 0.05

        # subscribers for the sensors

        # lidar 
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data) 

        # bumper
        self.hazard_subscription= self.create_subscription(HazardDetectionVector, 'hazard_detection', self.hazard_callback, qos_profile_sensor_data) 

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


        self.filtered_scan = [] # store filtered scan 
        
        self.SECTION_LIMITS = [(-pi/8, pi/8), (pi/8, 3/8*pi), (3/8*pi, 5/8*pi),
                               (5/8*pi, 7/8*pi), (-pi/8, -3/8*pi), (-3/8*pi, -5/8*pi),
                               (-5/8*pi, -7/8*pi), (-7/8*pi, 7/8*pi)] # section limits to split the scan readings
        

        # list of bumper areas
        self.bumper_areas =  {
            "bump_right" : False,
            "bump_front_right": False,
            "bump_front_center": False,
            "bump_front_left": False,
            "bump_left": False
        }


        self.bumper_areas_triggered = set() # save bumper areas triggered at every iteraction
        

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

        
        # initialize ANN
        self.INPUT_SIZE = 5 + 8 + 1 + 4
        self.HIDDEN_SIZE = 8
        self.OUTPUT_SIZE = 2 # linear velocity, angular velocity
        self.ann_controller = ANN_controller(self.INPUT_SIZE, self.HIDDEN_SIZE, self.OUTPUT_SIZE)


        self.enable_controller = False # this variable becomes true when all the sensor values are not None


    def hazard_callback(self, hazard_msg): # care only about bumper collision

        self.bumper_areas_triggered.clear()

        if not hazard_msg.detections: # if there aren't hazard detected
            for bumper_index in self.bumper_areas:
                self.bumper_areas[bumper_index] = False
        
        # build the set of bumper areas wich were triggered 
        else:
            for hazard in hazard_msg.detections:
                if hazard.type == 1: # this means bumper triggered
                    self.bumper_areas_triggered.add(hazard.header.frame_id)
            
            # check the bumper area triggered and update the dictionary
            for bumper_area in self.bumper_areas:
                self.bumper_areas[bumper_area] = bumper_area in self.bumper_areas_triggered


        self.publish_twist()



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

        self.publish_twist()



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
            self.publish_twist()


    # cliff sensors callbacks
    def cliff_sideL_callback(self, cliff_message):
        self.cliff_sideL_value = cliff_message.data
        self.publish_twist()

    def cliff_sideR_callback(self, cliff_message):
        self.cliff_sideR_value = cliff_message.data
        self.publish_twist()
    
    def cliff_frontL_callback(self, cliff_message):
        self.cliff_frontL_value = cliff_message.data
        self.publish_twist()
    
    def cliff_frontR_callback(self, cliff_message):
        self.cliff_frontR_value = cliff_message.data
        self.publish_twist()



    
    def publish_twist(self):

        # neural network prediction
        lin_vel, ang_vel = self.get_target_vel()

        # create and populate Twist message
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel
        self.twist_publisher.publish(twist_msg)
        self.get_logger().info('\nLinear vel: "%s" \nAngular vel:  "%s"' % (twist_msg.linear.x, twist_msg.angular.z))

            
    def get_target_vel(self):
        
        self.enable_controller = all([
            self.filtered_scan      is not None,
            self.light_direction    is not None,
            self.cliff_sideL_value  is not None,
            self.cliff_sideR_value  is not None,
            self.cliff_frontL_value is not None,
            self.cliff_frontR_value is not None
        ])
        

 
        if self.enable_controller:
            lin_vel, ang_vel = self.ann_controller.forward(self.filtered_scan 
                                                       + list(self.bumper_areas.values())
                                                       + [self.light_direction]  
                                                       + [self.cliff_frontL_value, self.cliff_frontR_value, self.cliff_sideL_value, self.cliff_sideR_value])

            print(lin_vel)
            print(ang_vel)

            lin_vel, ang_vel = self.apply_vel_threshold(lin_vel, ang_vel)
        
        else:
            lin_vel = 0.0
            ang_vel = 0.0


        return lin_vel, ang_vel


    # apply treshold on linear and angular velocities based on max velocities of the turtlebot4
    # and min velocities determined by the user to obtain no 0 target_vel
    def apply_vel_threshold(self, lin_vel, ang_vel):
        
        # for the linear velocity we want that the robot will be able to stop but not driving backwards
        if lin_vel > self.MAX_LIN_VEL:
            lin_vel = self.MAX_LIN_VEL
        elif lin_vel < self.MIN_LIN_VEL:
            lin_vel = 0.0
    

        # for the target velocity we want allow the robot to rotate clockwise and counter-clockwise and also to not rotate
        if abs(ang_vel) > abs(self.MAX_ANG_VEL):
            ang_vel = copysign(self.MAX_ANG_VEL, ang_vel)
        elif abs(ang_vel) < abs(self.MIN_ANG_VEL):
            ang_vel = 0.0

        return lin_vel, ang_vel


   
    
    def get_min_distance(self, scan_msg, min_index, max_index, switch_lectures):

        if switch_lectures:
            section_lectures = scan_msg.ranges[0:min_index] + scan_msg.ranges[max_index:]
        else: 
            section_lectures = scan_msg.ranges[min_index:max_index]


        # normalise lectures between 0 and 1
        normalized_lectures = [(x - scan_msg.range_min) / (scan_msg.range_max - scan_msg.range_min) for x in section_lectures]
        normalized_lectures = [scan_msg.range_max if x==float('inf') else x for x in normalized_lectures] # remove inf values
        

        return min(normalized_lectures)

    
    def get_reference_index(self, scan_msg, min_angle, max_angle):
                  
        min_angle = min_angle + pi
        max_angle = max_angle + pi

        msg_angle_min = scan_msg.angle_min + pi

        # compute correspondig index
        lower_index = int((min_angle - msg_angle_min) / scan_msg.angle_increment)
        upper_index = int((max_angle - msg_angle_min) / scan_msg.angle_increment)

        if lower_index > upper_index:
            lower_index, upper_index = upper_index, lower_index

        return lower_index, upper_index



def main(args=None):

    rclpy.init(args=args)

    controller_node = Controller_Node()
    

    # generate weights
    number_of_weights = (controller_node.INPUT_SIZE*controller_node.HIDDEN_SIZE + controller_node.HIDDEN_SIZE*controller_node.OUTPUT_SIZE)
    #weights = "0.5,"*number_of_weights + "0.5"

    #with open("/home/paolo/turtlebot4_controller_ws/src/turtlebot4_controller/turtlebot4_controller/param.txt", "w") as file:
    #    file.write(weights)

    # generate random weights
    #weights = [random.uniform(0, 1) for _ in range(number_of_weights)]

    # Scrivi i pesi nel file .txt
    #with open("/home/pleopardi/turtlebot4_controller_ws/src/turtlebot4_controller/turtlebot4_controller/param.txt", "w") as file:
        #file.write(",".join(map(str, weights)))

    # upload weights from param.txt
    param_path = '/home/pleopardi/turtlebot4_controller_ws/src/turtlebot4_controller/turtlebot4_controller/param.txt' # CHANGE if you are using swarm lab or personal laptop
    weights = open(param_path).read()
    weights = np.array(weights.split(','), np.float64)

    controller_node.ann_controller.upload_parameters(weights)

    rclpy.spin(controller_node)
 
    controller_node.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()