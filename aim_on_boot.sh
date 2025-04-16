#!/bin/bash

# Automatically get the current user's username
SERVICE_NAME="ros2_aiming_service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"

# Create the auto_aim_logs directory if it doesn't exist
LOG_DIR="/home/purduerm/auto_aim_logs"
mkdir -p "$LOG_DIR"

# Generate the log file name with current timestamp
LOG_FILE="$LOG_DIR/$(date +'%Y-%m-%d_%H-%M-%S')_auto_aim_log.txt"

# Command to run the service
COMMANDS_TO_RUN="source /opt/ros/humble/setup.bash && source /home/purduerm/ros2-ws/install/setup.bash && cd /home/purduerm/ros2-ws && ./auto-aiming/run run mv2control.py > $LOG_FILE 2>&1"

# Function to create the service
create_service() {
    echo "Creating the systemd service..."

    # Create the systemd service unit file
    cat <<EOL | sudo tee $SERVICE_FILE > /dev/null
[Unit]
Description=ROS 2 Auto Aiming Service
After=network.target

[Service]
ExecStart=/bin/bash -c "$COMMANDS_TO_RUN"
WorkingDirectory=/home/purduerm/ros2-ws
User=purduerm
Group=purduerm
Restart=always
Environment=ROS_DISTRO=humble
Environment=HOME=/home/purduerm

[Install]
WantedBy=multi-user.target
EOL

    # Reload systemd to recognize the new service
    sudo systemctl daemon-reload
}

# Function to enable the service to run on boot
enable_service() {
    echo "Enabling the service to start on boot..."
    sudo systemctl enable $SERVICE_NAME.service
    sudo systemctl start $SERVICE_NAME.service
    echo "Service started and enabled."
}

# Function to disable and remove the service
disable_service() {
    if [ -f "$SERVICE_FILE" ]; then
        echo "Disabling and removing the service..."
        sudo systemctl stop $SERVICE_NAME.service
        sudo systemctl disable $SERVICE_NAME.service
        sudo rm $SERVICE_FILE
        sudo systemctl daemon-reload
        echo "Service stopped, disabled, and removed."
    else
        echo "Service does not exist. Nothing to disable."
    fi
}

# Parse arguments
if [[ "$1" == "--disable" ]]; then
    disable_service
else
    # Create and enable the service by default
    create_service
    enable_service
fi
