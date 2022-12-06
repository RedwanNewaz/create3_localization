from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
import os
import sys

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    create3_traj_controller_dir = get_package_share_directory('create3_localization')
    # create3_traj_controller_dir = "/home/roboticslab/colcon_ws/src/create3_localization"
    # get path to params file
    params_path = os.path.join(
        create3_traj_controller_dir,
        'config',
        'ukf.yaml'
    )
    print(params_path)
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[params_path],
        ),
    ])
