import numpy as np 
# from nav_msgs.msg import Odometry

class LowpassFilter:
    def __init__(self, alpha, beta) -> None:
        self.position = None 
        self.orientation = None 
        self.alpha = alpha
        self.beta = beta 

    def __call__(self, *args, **kwds):

        self.position, self.orientation = self.convert(args[0], 3), self.convert(args[1], 4)

    def update(self, position, orientation):
        if self.position is None or self.orientation is None:
            self.position = position
            self.orientation = orientation
            return
        self.position = self.position * self.alpha + (1 - self.alpha) * position
        self.orientation = self.orientation * self.beta + (1 - self.beta) * orientation 
    
    def convert(self, arg, param):
        if param == 4:
            return np.array([arg.x, arg.y, arg.z, arg.w])
        else:
            return np.array([arg.x, arg.y, arg.z])

    # def to_nav_msg(self):
    #     odom_msg = Odometry()
    #     odom_msg.header.frame_id = "map"
    #     odom_msg.child_frame_id = "ukf/base_link"
    #     # odom_msg.child_frame_id = "odom"
    #     # odom_msg.header.stamp = self.get_clock().now().to_msg()

    #     odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = self.position
    #     odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w \
    #         = self.orientation
    #     return odom_msg


