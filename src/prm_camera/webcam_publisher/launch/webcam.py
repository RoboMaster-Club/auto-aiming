from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webcam_publisher',
            executable='VideoCaptureNode',
            parameters=[{'source': '/dev/video0',
                         'width': 1280,
                         'height': 720,
                         'fps': 30,
                         'frame_id': 'webcam',
                         'exposure': 25,
                         }]        
        )
    ])