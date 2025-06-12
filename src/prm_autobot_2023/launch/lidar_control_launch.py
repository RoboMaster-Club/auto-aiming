import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    autobot_dir = get_package_share_directory("prm_autobot_2023")

    
    control_comm_node = Node(package="control_communicator", executable="ControlCommunicatorNode")
    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(autobot_dir, "launch", "rplidar_range_limit_launch.py"))
            )

    return LaunchDescription([
        control_comm_node,
        lidar_launch,
        # lidar_launch
    ])