from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    webcam_publisher = get_package_share_path('webcam_publisher')
    video_path = webcam_publisher / 'resources/1.mp4'
    return LaunchDescription([
        Node(
            package='webcam_publisher',
            executable='VideoCaptureNode',
            parameters=[{'source': str(video_path),
                         'fps': 30,
                         'frame_id': 'video',
                         }]    
        )
    ])