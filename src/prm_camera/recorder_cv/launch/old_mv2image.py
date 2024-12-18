from launch import LaunchDescription
from launch_ros.actions import Node

from datetime import datetime

import sys
import os
from datetime import datetime

def get_fname():
    now = datetime.now()
    home = os.path.expanduser("~")

    dt = now.strftime("%Y-%m-%d-%H:%M:%S_")
    folder_name = dt + "mv"

    if not os.path.exists(home + "/recording"):
        os.mkdir(home + "/recording")

    os.mkdir(home + "/recording/" + folder_name)

    return os.path.join(home + "/recording/", folder_name + "/")

def get_frame_count():
    for arg in sys.argv:
        if "frame_count:=" in arg:
            return int(arg.split(":=")[1])
    return 0


def generate_launch_description():

    dst = get_fname()
    frame_count = get_frame_count()
    return LaunchDescription([
        Node(
            package="recorder",
            executable="VideoRecorderNode",
            parameters=[{
                "topic": "/image_raw", 
                "width": 1280, 
                "height": 1024, 
                "fps": 60, 
                "dst": dst,
                "frame_count": frame_count,
                "to_image" : 1
                }],
        )
    ])
