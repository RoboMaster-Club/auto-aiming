from launch import LaunchDescription
from launch_ros.actions import Node

from datetime import datetime
import random

import sys
import os
from datetime import datetime

def get_fname():
    now = datetime.now()
    home = os.path.expanduser("~")
    recording_dir = home + "/recording"

    dt = now.strftime("%Y-%m-%d-%H:%M:%S_")

    if not os.path.exists(recording_dir):
        os.mkdir(recording_dir)

    folders = [folder for folder in os.listdir(recording_dir) if os.path.isdir(os.path.join(recording_dir, folder))]
    folder_count = len(folders)

    random_number = str(random.randint(100000, 999999))  # Generate a random 6-digit number

    folder_name = dt + "mv" + str(folder_count) + "_" + random_number

    folder_path = os.path.join(recording_dir, folder_name)
    os.mkdir(folder_path)

    return folder_path + "/"

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
