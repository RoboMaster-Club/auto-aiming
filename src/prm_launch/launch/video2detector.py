from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

import os

def generate_launch_description():
    webcam_publisher = get_package_share_path('webcam_publisher')
    video_path = "/home/tom/Videos/close.avi" # example, can change to your liking
    return LaunchDescription([
        Node(
            package='webcam_publisher',
            output='screen',
            emulate_tty=True,
            executable='VideoCaptureNode',
            parameters=[{'source': str(video_path),
                         'fps': 10,
                         'frame_id': 'video',
                         }]    
        ), 
        Node(
            package='opencv_armor_detector',
            executable='OpenCVArmorDetectorNode',  
        )
    ])
