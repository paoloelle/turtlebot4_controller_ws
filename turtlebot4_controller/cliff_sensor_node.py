import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.time
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

# this node emulate a cliff sensor, two basic solution can be implemeted:
# 1- read the position of the robot and output a reading that can be seen like the average reading of the four sensors
# 2- create the tf static (one for each cliff sensor) like for the light sensors and based on the position of each sensor output a value

class Cliff_Sensor(Node):

    # limits of the 4 different areas
    nest_lim = 0.5
    cache_lim = 1.5
    slope_lim = 2.6

    def __init__(self):


        self.cliff_sensors_list = ['cliff_sensor_side_left', 'cliff_sensor_side_right', 'cliff_sensor_front_left', 'cliff_sensor_front_right']
        self.base_frame = 'arena' # to get the position related to the arena

        super().__init__('cliff_sensor_node')

        #subscriber (this holds only for implementation 1)
        #self.odom_subscription = self.create_subscription(
        #    PoseArray,
        #    '/model/turtlebot4/pose',  
        #    self.pose_callback,
        #    qos_profile_sensor_data            
        #)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

        # publisher (this holds only for implementation 1)
        #self.intensity_publisher = self.create_publisher(UInt16, 'cliff_intensity', qos_profile_sensor_data)

        # create publishers
        self.cliff_publishers = {}
        for cliff in self.cliff_sensors_list:
            self.cliff_publishers[cliff] = self.create_publisher(Float32, cliff, 1)

    
    def on_timer(self):

        for cliff in self.cliff_sensors_list:

            target_frame = cliff

            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, # reference frame
                    target_frame, # child frame
                    rclpy.time.Time()
                )

            except TransformException as ex:
                self.get_logger().info(f'Could not transform {target_frame} to {self.base_frame}: {ex}')
                return

            y_pose = t.transform.translation.y

            #self.get_logger().info("%s" % t)


            intensity_msg = Float32()

            if y_pose <= Cliff_Sensor.nest_lim:
                intensity_msg.data = 0.2
            elif y_pose <= Cliff_Sensor.cache_lim:
                intensity_msg.data = 0.4
            elif y_pose <= Cliff_Sensor.slope_lim:
                intensity_msg.data = 0.6
            else:
                intensity_msg.data = 0.8

            self.cliff_publishers[cliff].publish(intensity_msg)




    # this holds only for implementation 1
    #def pose_callback(self, pose_msg):
    #    
    #    y_pose = pose_msg.poses[0].position.y

    #    intensity_msg = UInt16()

    #    if y_pose <= Cliff_Sensor.nest_lim:
    #        intensity_msg.data = 1000
    #    elif y_pose <= Cliff_Sensor.cache_lim:
    #        intensity_msg.data = 2000
    #    elif y_pose <= Cliff_Sensor.slope_lim:
    #        intensity_msg.data = 3000
    #    else:
    #        intensity_msg.data = 4000

    #    self.intensity_publisher.publish(intensity_msg)


def main(args=None):
    rclpy.init(args=args)
    cliff_sensor_node = Cliff_Sensor()
    rclpy.spin(cliff_sensor_node)
    cliff_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()