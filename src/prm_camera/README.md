# prm_camera

### Software to interface with our MindVision Industrial Camera on the robots. 

- `mv_publisher`:
    - We use a [MV-SUA133GC Industrial Camera](https://www.mindvision.com.cn/cpzx/info_62.aspx?itemid=1996&parent&lcid=101) on our robots for vision. This node utilizes the MindVision API to read from the camera, set camera parameters, and publish frames on `/image_raw`.
- `recorder_cv`: 
    - Subscribes to the `/image_raw` camera feed and writes the received frames to a video file (lossy or lossless). Very useful for recording matches or scenarios and replaying them through the CV pipeline later on.
- `webcam_publiser`: 
    - Similar to `mv_publisher`, but uses a USB webcam (`/dev/video0` by default) and publishes frames on `/image_raw`.
- `image_noise_adder`: 
    - Takes a frame feed on `/image_raw`, adds random noise to the frame, and republishes the frame on `/image_noise`. Mainly used for testing, not production.
