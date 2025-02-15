from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

import os

def generate_launch_description():
    webcam_publisher = get_package_share_path('webcam_publisher')
    video_path = "/home/user-accounts/public/spintop/moving_but_no_spinning.avi" # example, can change to your liking
    return LaunchDescription([
        Node(
            package='webcam_publisher',
            output='screen',
            emulate_tty=True,
            executable='VideoCaptureNode',
            parameters=[{'source': str(video_path),
                         'fps': 4,
                         'frame_id': 'video',
                         'cam_barrel_angle': 1.0;
                         }]    
        ), 
        Node(
            package='opencv_armor_detector',
            executable='OpenCVArmorDetectorNode',  
        ),
        Node(
            package='pose_estimator',
            executable='PoseEstimatorNode',
            parameters[{'cam_barrel_roll': 0.0, # camera -> barrel rotations and translations. (radians and mm)
                        'cam_barrel_pitch': 0.0, # angles are 0 bc camera is parallel at the moment
                        'cam_barrel_yaw': 0.0,
                        'cam_barrel_x': -88, # camera is 88 mm to left of barrel tip
                        'cam_barrel_y': 73, # camera is 73 mm above barrel tip
                        'cam_barrel_z': 80, # camera is 80 mm behind barrel tip
                        }]
        ),

        //TODO:: get actual angle, potentially set up the parameter to change based on control
    ])
