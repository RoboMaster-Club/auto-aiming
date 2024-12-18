# prm_launch

### Launchfiles to automatically start Algorithm Team's ROS2 computer vision pipeline

***

### Running the Computer Vision Pipeline at Home:
1. Clone all the relevant projects into your `~/ros2-ws/src/` directory (skip if you already did this).
2. Go into `~/ros2-ws` and build all the projects: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O3 -DCMAKE_CXX_FLAGS=-DPRODUCTION` 
3. If you are running on your own dev server account, you will likely want to feed a video file into the pipeline. If so, you will be using a **launchfile** called `video2pnp.py`, located inside `prm_launch/launch/video2pnp.py`. Edit this file, and modify the following line to the path of the video you want to feed into the pipeline:

    ```
    video_path = "/home/tom/Videos/example.avi"
    ```
    I have some example videos in `/home/user-accounts/public/`.  You may also change the FPS or anything to your liking. 
    
    Once you're satisfied, save the file and run `ros2 launch prm_launch video2pnp.py` to start the pipeline. You should now see several windows pop up displaying your video, and all our code should be running on your video.  

    *Tip: You can also run `ros2 launch prm_launch mv2pnp.py` to use a MindVision camera instead of a video file.*

***

### List of other Launchfiles:

- **mv2control.py**:
    - **Launches the full pipeline from camera to control communication**. Launches `mv_publisher`, `opencv_armor_detector`, `pnp_solver`, and `control_communicator`. Mainly useable when running on an actual robot.
- **video2pnp.py:**
    - **The most helpful script if running pipeline on the development server**. Reads a video file and feeds it into `opencv_armor_detector` and `pnp_solver`. launches `VideoCaptureNode`, `opencv_armor_detector`, and `pnp_solver`.
- **mv2pnp.py**:
    - Launches `mv_publisher`, `opencv_armor_detector`, and `pnp_solver`.
- **mv2detection.py**:
    - Launches `mv_publisher` and `opencv_armor_detector`.
- **video2noise2pnp.py:** 
    - Launches `VideoCaptureNode`, `image_noise_adder`, and `opencv_armor_detector`.
- **webcam_autoaim.py:**
    - Runs auto-aim pipeline using webcam instead of MindVision camera. Launches `webcam_publisher`, `opencv_armor_detector`, `pnp_solver`, and `control_communicator`.


### Running Individual Launchfiles:
After `colcon build`ing all relevant nodes, run `ros2 launch prm_launch <launchfile>` where `<launchfile>` is one of the above files. All the nodes listed in the launchfile will be started. 
