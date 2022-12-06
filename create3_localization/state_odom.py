import rclpy
from rclpy.node import Node
from rclpy.qos import QoSLivelinessPolicy, QoSDurabilityPolicy
import numpy as np 

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from copy import deepcopy



qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
class OdomState(Node):
    def __init__(self):
        super().__init__("odom_state")
        self.__refApriltagCoord = None 
        self.__refOdomCoord = None 

        self.apriltag_odom_sub = self.create_subscription(Odometry, '/apriltag/odom', self.apriltag_odom_callback, 10)
        self.create3_odom_sub = self.create_subscription(Odometry, '/odom', self.create3_odom_callback, qos_policy)
        self.pub_odom = self.create_publisher(Odometry, '/sync/odom', 10)


    def apriltag_odom_callback(self, msg: Odometry):
        position = self.get_odom_position_array(msg)
        # print('[+] apriltag position ', position)
        if self.__refApriltagCoord is None:
            self.__refApriltagCoord = position
        

    def create3_odom_callback(self, msg: Odometry):
 
        if self.__refApriltagCoord is None:
            return
        position = self.get_odom_position_array(msg)
        
        if self.__refOdomCoord is None:
            self.__refOdomCoord = self.__refApriltagCoord - position
        
        new_msg = deepcopy(msg)
        new_msg.header.frame_id = "map"
        new_msg.child_frame_id = "ukf/base_link"

        rel_position = position + self.__refOdomCoord
        print(rel_position)
        new_msg.pose.pose.position = self.array_to_point(rel_position)
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