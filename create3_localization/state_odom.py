import rclpy
from rclpy.node import Node
import numpy as np 

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from copy import deepcopy

class OdomState(Node):
    def __init__(self):
        super().__init__("odom_state")
        self.__refApriltagCoord = None 
        self.__refOdomCoord = None 

        self.declare_parameter('apriltag_odom', 'apriltag/odom')
        self.declare_parameter('create3_odom', 'odom')
        self.declare_parameter('pub_odom', 'sync/odom')

        apriltag_odom = self.get_parameter('apriltag_odom').get_parameter_value().string_value
        create3_odom = self.get_parameter('create3_odom').get_parameter_value().string_value
        pub_odom = self.get_parameter('pub_odom').get_parameter_value().string_value


        self.apriltag_odom_sub = self.create_subscription(Odometry, apriltag_odom, self.apriltag_odom_callback, 10)
        self.create3_odom_sub = self.create_subscription(Odometry, create3_odom, self.create3_odom_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, pub_odom, 10)


    def apriltag_odom_callback(self, msg: Odometry):
        position = get_odom_position_array(msg)
        if self.__refApriltagCoord is None:
            self.__refApriltagCoord = position
        

    def create3_odom_callback(self, msg: Odometry):
        if self.__refApriltagCoord is None:
            return

        position = get_odom_position_array(msg)
        if self.__refOdomCoord is None:
            self.__refOdomCoord = self.__refApriltagCoord - position
        
        new_msg = deepcopy(msg)
        new_msg.header.frame_id = "map"
        new_msg.child_frame_id = "ukf/base_link"
        new_msg.pose.pose.position = self.array_to_point(position + self.__refOdomCoord)
        self.pub_odom.publish(new_msg)

    @staticmethod
    def get_odom_position_array(msg:Odometry):
        position = msg.pose.pose.position
        return np.array([position.x, position.y, position.z])
    
    @staticmethod
    def array_to_point(arr):
        p = Point()
        p.x, p.y, p.z = arr
        return p
        

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = OdomState()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()