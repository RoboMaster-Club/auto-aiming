from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

import os

def generate_launch_description():
    webcam_publisher = get_package_share_path('webcam_publisher')
    # video_path = "/home/tom/Videos/far_back_spin_and_move.avi" # example, can change to your liking
    video_path = "/home/user-accounts/lee4649/auto-aiming/videos/spinning_in_place.avi"
    return LaunchDescription([
        Node(
            package='webcam_publisher',
            output='screen',
            emulate_tty=True,
            executable='VideoCaptureNode',
            parameters=[{'source': str(video_path),
                         'fps': 24,
                         'frame_id': 'video',
                         }]    
        ), 
        Node(
            package='opencv_armor_detector',
            executable='OpenCVArmorDetectorNode',  
        ),
        Node(
            package='pnp_solver',
            executable='PNPSolverNode',  
        ),
        Node(
            package='control_communicator',
            executable='ControlCommunicatorNode',  
        )
    ])
