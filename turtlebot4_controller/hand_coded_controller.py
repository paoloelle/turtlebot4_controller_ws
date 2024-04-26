import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from math import atan2, tan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from math import pi, degrees, copysign
import random
from statistics import mean
from irobot_create_msgs.msg import HazardDetectionVector




class Controller_Node(Node):

    def __init__(self):

        super().__init__('hand_coded_controller')

        #turtlebot4 max velocities
        self.MAX_ANG_VEL = 1.90 # 1.90 rad/s
        self.MAX_LIN_VEL = 0.46 # 0.46 m/s

        # threshold to determine a target velocity different from 0
        self.MIN_ANG_VEL = 0.1
        self.MIN_LIN_VEL = 0.05

        # subscribers for the sensors

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


        # FSM control variable
        self.on_source = False
        self.on_cache = False
        self.on_nest = True
        self.on_slope = False
        self.pushed_obj = False

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

        if any(self.bumper_areas.values()):
            self.pushed_obj = True



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
        self.get_area()
        
    def cliff_sideR_callback(self, cliff_message):
        self.cliff_sideR_value = cliff_message.data
        self.get_area()
    
    def cliff_frontL_callback(self, cliff_message):
        self.cliff_frontL_value = cliff_message.data
        self.get_area()
    
    def cliff_frontR_callback(self, cliff_message):
        self.cliff_frontR_value = cliff_message.data
        self.get_area()

    def get_area(self):
        
        if self.cliff_frontL_value and self.cliff_frontR_value and self.cliff_sideL_value and self.cliff_sideR_value:

            avg_cliff_value = round(mean([self.cliff_frontL_value, self.cliff_frontR_value,
                                    self.cliff_sideL_value, self.cliff_sideR_value]), 1)

            if avg_cliff_value == 0.2:
                self.on_nest = True
                self.on_cache = False
                self.on_slope = False
                self.on_source = False

            elif avg_cliff_value == 0.3:
                self.on_nest = False
                self.on_cache = True
                self.on_slope = False
                self.on_source = False

            elif avg_cliff_value == 0.4:
                self.on_nest = False
                self.on_cache = False
                self.on_slope = True
                self.on_source = False

            elif avg_cliff_value == 0.5:
                self.on_nest = False
                self.on_cache = False
                self.on_slope = False
                self.on_source = True



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


            # FSM starts here

            if self.on_nest: # GO TO SOURCE (anti phototaxis)
                lin_vel = 0.3
                ang_vel = 0.1*-self.light_direction
                self.get_logger().info('GO TO SOURCE n')
                self.pushed_obj = False

            elif self.pushed_obj: # GO TO NEST (phototaxis)
                lin_vel = 0.3
                ang_vel = 0.1*self.light_direction
                self.get_logger().info('GO TO NEST')
    
            elif self.on_source: # RANDOM WALK
                lin_vel = random.uniform(self.MIN_LIN_VEL, 0.3*self.MAX_LIN_VEL)
                ang_vel = random.uniform(-self.MAX_ANG_VEL, self.MAX_ANG_VEL)
                self.get_logger().info('RANDOM WALK')

            elif not self.on_source: # GO TO SOURCE (anti phototaxis)
                lin_vel = 0.3
                ang_vel = 0.6*-self.light_direction
                self.get_logger().info('GO TO SOURCE')


            lin_vel, ang_vel = self.apply_vel_threshold(lin_vel, ang_vel)
            twist_msg.linear.x = lin_vel
            twist_msg.angular.z = ang_vel

        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        
        self.twist_publisher.publish(twist_msg)

        self.get_logger().info('\nLinear vel: "%s" \nAngular vel:  "%s"' % (twist_msg.linear.x, twist_msg.angular.z))
        


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
            

    
def main(args=None):

    rclpy.init(args=args)

    controller_node = Controller_Node()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

