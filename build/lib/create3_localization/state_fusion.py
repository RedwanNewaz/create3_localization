#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry
from . import StateEstimator, Measurement
import numpy as np 
import math 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion



class Localizer(Node):
    def __init__(self, from_frame, to_frame, pub_topic):
        super().__init__('state_fusion')
        print("localizer node started")

          # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          to_frame, from_frame).get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Call on_timer function every second
        self.timer = self.create_timer(0.015, self.on_timer)
        self.pub_odom = self.create_publisher(Odometry, pub_topic, 10)

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.from_frame_rel = from_frame
        self.to_frame_rel = to_frame
        self.init_transform = None 
        self.state = StateEstimator()
        self.cmd_vel = (0, 0)
    
    def cmd_vel_callback(self, msg:Twist):
        self.cmd_vel = (msg.linear.x, msg.angular.z)
        

    def transform_to_odom(self, position, orientation):
        
        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "ukf/base_link"
        # odom_msg.child_frame_id = "odom"
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = position
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w \
            = orientation.x, orientation.y, orientation.z, orientation.w 


        return odom_msg  

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.from_frame_rel,
                self.to_frame_rel,
                rclpy.time.Time())
            q = t.transform.rotation
            
            (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            theta = np.pi / 2.0
            yaw = np.pi - (yaw + theta + np.pi) % (2 * np.pi)
            
            # transform coordinates and rotation 
            q = self.quaternion_from_euler(0, 0, yaw)
            p = t.transform.translation
            p.x, p.y = -p.y, -p.x
            
            # update extended kalman filter
            new_transform = Measurement.get_transform_from_tf_msg(p, q)
            z = Measurement.from_transformation_matrix(new_transform)
            x = self.state(self.cmd_vel[0], self.cmd_vel[1], z)
            print('state = ', x)

            pp, __ = self.state.to_msg()
            qq = self.quaternion_from_euler(0, 0, x[-1])
            msg = self.transform_to_odom(pp, qq)
            self.pub_odom.publish(msg)
            # print(msg)
        except TransformException as ex:
            # self.get_logger().info(
            #     f'Could not transform {self.to_frame_rel} to {self.from_frame_rel }: {ex}')
            return
    
    
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
     

def main(args=None):
    rclpy.init(args=args)
    from_frame = "camera"
    to_frame = "tag36h11:7"
    pub_topic = 'apriltag/odom'
    minimal_subscriber = Localizer(from_frame, to_frame, pub_topic)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    