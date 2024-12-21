# auto-aiming
Revitalized auto-aiming suite for Purdue RoboMaster Club 2024-2025.

## Overall Suite Requirements
### Functional Requirements:
- [ ] **Detect an enemy armor plate in the camera's FOV.**
  - [ ] Meet the following detection rate and accuracy requirements:
    - [ ] 5 meters: 90% detection rate, 5% pixel loss
    - [ ] 3 meters: 95% detection rate, 5% pixel loss
    - [ ] 2 meters: 95% detection rate, 5% pixel loss
  - [ ] Reduce search area around previously detected plates ("search area reduction").
  - [ ] Achieve 120 Hz detection frequency.
  - [ ] Classify the robot type based on its armor plate sticker.
- [ ] **Compute camera-relative XYZ pose via PnP solving with 5% error margin.**
- [ ] **Filter out false positives and noise in the detection results.**
  - [ ] Use a Kalman filter to smooth XYZ pose estimates.
  - [ ] Apply a "validity filter" for erroneous detection/pose results (e.g., based on distance, XYZ shifts). 
- [ ] **Compute the gimbal angles (yaw and pitch) required to accurately land projectiles on the detected armor plate.**
  - [ ] Compute pitch using an easily-adjustable lookup table or ballistic model based on distance to target.
  - [ ] Compute yaw using a predictive model using the detected armor's XYZ pose, rotation, and velocity
- [ ] **Send the computed gimbal angles to the STM32 control board via UART.**

### Non-Functional Requirements:
- [ ] **Performance**  
  - Ensure real-time end-to-end performance of 120 Hz.
- [ ] **Testability**  
  - Include a comprehensive suite of unit tests for all modules to verify component correctness.
- [ ] **Maintainability**  
  - Maintain modularity by separating ROS2 and C++ logic into `xyzNode.cpp` and `xyz.cpp` files.  
  - Provide thorough documentation, including doxygen-style comments for functions and README files for modules.



## Usage  
Clone this repository into your `ros2-ws` directory. We provide a `run` script that can be used to build, run, test, and clean the workspace with the following options:

- **Build**
  - `./auto-aiming/run build`
- **Launch ROS2 code**
  - `./auto-aiming/run run <launch_file>`
- **Run automated tests (GTest)**
  - `./auto-aiming/run test`
- **Clean the workspace**
  - `./auto-aiming/run clean`
- _Optional flags_
  - `--quiet`: Suppresses console output, logs output to `command_output.log`.
  - `--debug`: Builds with debug flags enabled.

### Example:
```
./auto-aiming/run --debug --quiet run video2detector.py
```

## Architecture Diagram

<div style="max-width: 600px; margin: auto;">
    <img src="https://user-content.gitlab-static.net/e4204bbed045ad52aa41d39922ba810a488a8b23/68747470733a2f2f6769746875622e636f6d2f526f626f4d61737465722d436c75622f507572647565524d2d57696b692f626c6f622f67682d70616765732f646f63732f616c676f726974686d2f7265736f75726365732f616c677465616d706c6f742e6a70673f7261773d74727565" alt="alt text">
</div>
