import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    
    map_transform = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["1.892", "-0.726", "1.278", "-2.4337972", "-0.2495326", "-2.7776417",  "camera", "map"])
    
    odom_transform = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"])
    
    # robot frame converstion 
    robot_frame = Node(package="create3_localization", 
        executable='robot_state',
        name='robot_state_estimator'
        

    )

    viz_node = Node(
        name='apriltag_36h11Viz',
        package="create3_localization",
        executable="state_viz"
    )

    odom_node = Node(
        name='odom_sync_state',
        package="create3_localization",
        executable="sync_state"
        
    )

    
    current_pkg_dir = get_package_share_directory("create3_localization")
    sensor_fusion = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                current_pkg_dir + '/launch/ukf.launch.py'))

    # return launch.LaunchDescription([robot_frame, odom_node, odom_transform, viz_node, odom_transform])
    # return launch.LaunchDescription([map_transform, odom_transform, odom_node, robot_frame, viz_node, sensor_fusion])
    return launch.LaunchDescription([map_transform, odom_transform, odom_node, robot_frame, viz_node])
