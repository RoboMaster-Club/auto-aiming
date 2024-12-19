# auto-aiming
Revitalized auto-aiming suite for Purdue RoboMaster Club 2024-2025.

## Usage  
To use the run script, execute the following commands in your terminal. The script supports multiple options:

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
`./run --debug --quiet run video2detector.py`
