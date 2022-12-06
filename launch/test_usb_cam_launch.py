import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_localization")
    #default_ros_cam_dir = "file:///home/robo/.ros/camera_info/head_camera_nexigo_1920.yaml"
    nexigo_cam = {
        "camera_name" : 'nexigo_cam',
        "camera_info_url": "file://{}/config/head_camera_nexigo_1920.yaml".format(current_pkg_dir),
        "framerate" : 60.0,
        "frame_id" : "camera",
        "image_height"  : 1080,
        "image_width"   : 1920,
        "io_method"     : "mmap",
        "pixel_format"  : "mjpeg",
        # "color_format"  : "yuv422p",
        "video_device"  : "/dev/video0"
    }

    # cam_node = ComposableNode(
    #     namespace='camera',
    #     package='usb_cam', plugin='usb_cam::UsbCamNode',
    #     parameters=[nexigo_cam],
    #     extra_arguments=[{'use_intra_process_comms': True}],
    # )

    cam_node = Node(
        namespace='camera',
        package='usb_cam', executable='usb_cam_node_exe',
        parameters=[nexigo_cam]
    )

    viz_cam =  Node(
        package='usb_cam', executable='show_image.py', output='screen',
        # namespace=ns,
        # arguments=[image_manip_dir + "/data/mosaic.jpg"])
        # remappings=[('image_in', 'image_raw')]
        )

    return launch.LaunchDescription([cam_node, viz_cam])
