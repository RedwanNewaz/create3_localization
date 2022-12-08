import rclpy
from rclpy.node import Node
from rclpy.qos import QoSLivelinessPolicy, QoSDurabilityPolicy
import numpy as np 
import math 

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion 
from copy import deepcopy
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .low_pass_filter import LowpassFilter
from . import StateEstimator, Measurement



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

        self.__min_samples = 10
        

        self.apriltag_lpf = LowpassFilter(0.9, 0.9)
        self.state = StateEstimator()
        self.cmd_vel = (0, 0)


    def apriltag_odom_callback(self, msg: Odometry):
        if self.__min_samples < 0:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            new_transform = Measurement.get_transform_from_tf_msg(p, q)
            z = Measurement.from_transformation_matrix(new_transform)
            x = self.state(self.cmd_vel[0], self.cmd_vel[1], z)
            return

        if self.__min_samples >= 0:
            self.apriltag_lpf(msg.pose.pose.position, msg.pose.pose.orientation)
        
        if self.__min_samples < 1 and  self.__refApriltagCoord is None:
            self.__refApriltagCoord = self.apriltag_lpf.position
            self.get_logger().info("[+] Apriltag Low Pass Filtered Initialized")
        
        self.__min_samples -= 1 

    
    def transform_to_odom(self, position, orientation):
        
        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "ukf/base_link"
        # odom_msg.child_frame_id = "odom"
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        if isinstance(position, np.ndarray):
            odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = position 
        else:
            odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = position.x, position.y, position.z
            
        if isinstance(orientation, np.ndarray):
            odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = orientation
        else:
            odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w \
                = orientation.x, orientation.y, orientation.z, orientation.w 

        return odom_msg  

   
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q 
        

    def create3_odom_callback(self, msg: Odometry):
 
        if self.__refApriltagCoord is None:
            return
        position = self.get_odom_position_array(msg)
        if self.__refOdomCoord is None:
            self.__refOdomCoord = self.__refApriltagCoord - position

        
        # orientation should be rotated 90 degree CW

        q = msg.pose.pose.orientation  
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        theta = -np.pi / 2.0
        yaw = (yaw + theta + np.pi) % (2 * np.pi) - np.pi
        
        # transform coordinates and rotation 
        q = self.quaternion_from_euler(0, 0, yaw)
        p = msg.pose.pose.position

        p.x, p.y = (self.__refOdomCoord[0] + p.x), (self.__refOdomCoord[1] + p.y)

        new_transform = Measurement.get_transform_from_tf_msg(p, q)
        z = Measurement.from_transformation_matrix(new_transform)
        x = self.state(self.cmd_vel[0], self.cmd_vel[1], z)

        pp, _ = self.state.to_msg()
        #use odom orientation only (tag orientation is erroneous)
        new_msg = self.transform_to_odom(pp, q)
        

        self.pub_odom.publish(new_msg)
    
    @staticmethod
    def rotate_robot(arg, theta):
        curr_heading = [arg.x, arg.y, arg.z, arg.w]
        (roll, pitch, yaw) = euler_from_quaternion(curr_heading) 
        yaw = (yaw + theta + math.pi) % (2 * math.pi) - math.pi 
        orientation_q = quaternion_from_euler(0, 0, yaw)
        q = Quaternion()
        # ryxz
        q.w, q.y, q.x, q.z= orientation_q
        return q

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