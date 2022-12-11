import rclpy
from rclpy.node import Node
import math
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry 
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import cv2

class ApriltagSubscriber(Node):

    def __init__(self):
        super().__init__('apriltag_subscriber')
        print("apriltag subscriber node started")

        self.br = CvBridge()
        self.__image = None
        self.pub_img = self.create_publisher(Image, "tag_detections/image_raw", 10)
        self.pub_marker = self.create_publisher(Marker, "/create3_state", 10)

        self.declare_parameter('kf_topic', 'apriltag/odom')
        kf_topic = self.get_parameter('kf_topic').get_parameter_value().string_value


        self.tag_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.apriltag_detection_callback,
            10)
        self.tag_subscription  # prevent unused variable warning

        # image subscription 
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw',
            self.camera_image_callback,
            10
        )
        self.image_subscription

        # kf filter subscription 
        self.kf_subscription = self.create_subscription(
            Odometry, kf_topic,
            self.ukf_callback,
            10
        )
        self.kf_subscription

    def ukf_callback(self, msg):
        def convert(arg, param):
            return [arg.x, arg.y, arg.z, arg.w] if param == 4 else [arg.x, arg.y, arg.z]         

        position = convert(msg.pose.pose.position, 3)
        orientation = convert(msg.pose.pose.orientation, 4)

        marker = self.getMarkerWindow(position, orientation)
        self.pub_marker.publish(marker)


        

    def annotate_tag(self, img, info):

        name = f"{info.id}"
        # Line thickness of 2 px
        thickness = 2

        font = cv2.FONT_HERSHEY_SIMPLEX
        # org
        org = (math.ceil(info.centre.x), math.ceil(info.centre.y))
        # fontScale
        fontScale = 1
        # Blue color in BGR
        color = (0, 255, 0)
        img = cv2.putText(img, name, org, font,
                            fontScale, color, thickness, cv2.LINE_AA)
        return img


    def apriltag_detection_callback(self, msg):

        if self.__image is None:
            return

        # self.get_logger().info(f"{msg}")
        image = self.__image.copy()
        for tag in msg.detections:
            # print(tag.centre)
            self.annotate_tag(image, tag)

        self.pub_img.publish(self.br.cv2_to_imgmsg(image))

        # cv2.imshow("camera", self.__image)
        # cv2.waitKey(1)

    def camera_image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.__image = self.br.imgmsg_to_cv2(msg)



    def getMarkerWindow(self, p, q):

        myMarker = Marker()
        myMarker.header.frame_id = "map"
        # myMarker.header.seq = 1
        myMarker.header.stamp    = self.get_clock().now().to_msg()
        myMarker.ns = "robot"
        myMarker.id = 1
        myMarker.type = myMarker.MESH_RESOURCE # sphere
        # myMarker.type = myMarker.ARROW # sphere
        myMarker.action = myMarker.ADD
        myMarker.pose.position.x = p[0]
        myMarker.pose.position.y = p[1]
        myMarker.pose.position.z = p[2]

        # rotate 180 degree
        q = self.rotate_robot(q, math.pi)
        myMarker.pose.orientation.x=q[0]
        myMarker.pose.orientation.y=q[1]
        myMarker.pose.orientation.z=q[2]
        myMarker.pose.orientation.w=q[3]
        myMarker.mesh_resource = "package://irobot_create_description/meshes/body_visual.dae";
        myMarker.mesh_use_embedded_materials = True
        myMarker.color.r = 0.663
        myMarker.color.g = 0.663
        myMarker.color.b = 0.663
        myMarker.color.a = 0.89
        # myMarker.scale.x = 0.2
        # myMarker.scale.y = 0.2
        # myMarker.scale.z = 0.2
        myMarker.scale.x = 1.0
        myMarker.scale.y = 1.0
        myMarker.scale.z = 1.0

        return myMarker 
    @staticmethod
    def rotate_robot(curr_heading, theta):
        (roll, pitch, yaw) = euler_from_quaternion(curr_heading) 
        yaw = (yaw + theta + math.pi) % (2 * math.pi) - math.pi 
        orientation_q = quaternion_from_euler(0, 0, yaw)
        return orientation_q

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ApriltagSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()