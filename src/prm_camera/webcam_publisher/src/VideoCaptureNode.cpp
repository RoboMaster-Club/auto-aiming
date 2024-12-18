// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include "VideoCaptureNode.hpp"

VideoCaptureNode::VideoCaptureNode(const rclcpp::NodeOptions &options) : Node("video_capture", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting Webcam Publisher Node");

    // Declare camera parameters
    declare_parameters();

    // Create camera publisher
    // rqt_image_view can't subscribe image msg with sensor_data QoS
    // https://github.com/ros-visualization/rqt/issues/187
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub = image_transport::create_camera_publisher(this, "image_raw", qos);

    seq = 0;

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "video_capture");
    camera_info_manager = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter("camera_info_url", "package://webcam_publisher/config/webcam_info.yaml");

    if (camera_info_manager->validateURL(camera_info_url))
    {
        camera_info_manager->loadCameraInfo(camera_info_url);
        camera_info_msg = camera_info_manager->getCameraInfo();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    // Add callback to the set parameter event
    params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&VideoCaptureNode::parameters_callback, this, std::placeholders::_1));
    // std::cout <<

    frame = cv::Mat(height, width, CV_8UC3);

    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&VideoCaptureNode::capture_frame, this));
    capture_thread = std::thread(std::bind(&VideoCaptureNode::capture_frame, this));
}

VideoCaptureNode::~VideoCaptureNode()
{
    cap.release();
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

void VideoCaptureNode::capture_frame()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Starting threads");

    rclcpp::WallRate loop_rate(fps + 1);
    while (rclcpp::ok())
    {
        cap.read(frame);
        camera_info_msg.header.stamp = sensor_msg_header.stamp = this->now();
        sensor_msg_header.frame_id = frame_id + std::to_string(seq);

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "End of frame at %d", seq);
            cap.release();
            cap.open(source);
            RCLCPP_INFO(this->get_logger(), "Reopen Camera");
            continue;
        }

        image_msg = cv_bridge::CvImage(sensor_msg_header, "bgr8", frame).toImageMsg();

        camera_pub.publish(*image_msg, camera_info_msg);

        RCLCPP_INFO_ONCE(this->get_logger(), "First Frame Published");

        if (seq % 1000 == 0)
            RCLCPP_INFO(this->get_logger(), "seq: %d", seq);
        seq++;
        loop_rate.sleep();
    }
}

void VideoCaptureNode::declare_parameters()
{

    // Source
    source = this->declare_parameter("source", "/dev/video0");
    cap.open(source);
    RCLCPP_INFO(this->get_logger(), "Source = %s", source.c_str());
    if (cap.isOpened())
    {
        RCLCPP_INFO(this->get_logger(), "Open camera success!");
        if (source.compare("/dev/video0") == 0)
        {
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            RCLCPP_INFO(this->get_logger(), "Setting codec: MJPG");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Open camera failed!");
    }

    // FPS
    fps = this->declare_parameter("fps", 30);
    if (fps > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Setting FPS: %d", fps);

        if (!cap.set(cv::CAP_PROP_FPS, fps))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set FPS");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Current FPS: %d", cap.get(cv::CAP_PROP_FPS));
    }

    // Width and Height
    width = this->declare_parameter("width", 0);
    height = this->declare_parameter("height", 0);
    if (width > 0 && height > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Setting size: (%d, %d)", width, height);
        if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, width) || !cap.set(cv::CAP_PROP_FRAME_HEIGHT, height))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set resolution");
            width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        }
    }
    else
    {
        width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(this->get_logger(), "Current size: (%d, %d)", width, height);
    }

    // Width and Height
    exposure = this->declare_parameter("exposure", 0);
    if (exposure > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Exposure: %d", exposure);

        if (!cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1) || !cap.set(cv::CAP_PROP_EXPOSURE, exposure))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to set exposure");
        }
    }
    else
    {
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
        RCLCPP_INFO(this->get_logger(), "Auto exposure: %d", cap.get(cv::CAP_PROP_EXPOSURE));
    }

    // Frame ID
    std::string frame_id = this->declare_parameter("frame_id", "video_capture");
    RCLCPP_INFO(this->get_logger(), "Frame ID = %s", frame_id.c_str());
}

rcl_interfaces::msg::SetParametersResult VideoCaptureNode::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
        if (param.get_name() == "source")
        {
            cap.open(param.as_string());
            RCLCPP_INFO(this->get_logger(), "Source = %s", param.as_string().c_str());
            if (cap.isOpened())
            {
                RCLCPP_INFO(this->get_logger(), "Open source %s success!", param.as_string().c_str());
            }
            else
            {
                result.successful = false;
                RCLCPP_WARN(this->get_logger(), "Open camera %s failed!", param.as_string().c_str());
            }
        }
        else if (param.get_name() == "exposure")
        {
            if (param.as_int() == 0)
            {
                if (cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3))
                {
                    RCLCPP_INFO(this->get_logger(), "Set auto exposure on success!");
                }
                else
                {
                    result.successful = false;
                    RCLCPP_WARN(this->get_logger(), "Set auto exposure off failed!");
                }
            }
            else if (cap.set(cv::CAP_PROP_EXPOSURE, param.as_int()))
            {
                RCLCPP_INFO(this->get_logger(), "Set exposure %d success!", param.as_int());
            }
            else
            {
                result.successful = false;
                RCLCPP_WARN(this->get_logger(), "Set exposure %d failed!", param.as_int());
            }
        }
        else if (param.get_name() == "gain")
        {
            if (cap.set(cv::CAP_PROP_GAIN, param.as_int()))
            {
                RCLCPP_INFO(this->get_logger(), "Set gain %d success!", param.as_int());
            }
            else
            {
                result.successful = false;
                RCLCPP_WARN(this->get_logger(), "Set gain %d failed!", param.as_int());
            }
        }
        else if (param.get_name() == "fps")
        {
            if (cap.set(cv::CAP_PROP_FPS, param.as_int()))
            {
                RCLCPP_INFO(this->get_logger(), "Set fps %d success!", param.as_int());
            }
            else
            {
                result.successful = false;
                RCLCPP_WARN(this->get_logger(), "Set fps %d failed!", param.as_int());
            }
        }
        else if (param.get_name() == "width")
        {
            if (cap.set(cv::CAP_PROP_FRAME_WIDTH, param.as_int()))
            {
                RCLCPP_INFO(this->get_logger(), "Set width %d success!", param.as_int());
            }
            else
            {
                result.successful = false;
                RCLCPP_WARN(this->get_logger(), "Set width %d failed!", param.as_int());
            }
        }
        else if (param.get_name() == "height")
        {
            if (cap.set(cv::CAP_PROP_FRAME_HEIGHT, param.as_int()))
            {
                RCLCPP_INFO(this->get_logger(), "Set height %d success!", param.as_int());
            }
            else
            {
                result.successful = false;
                RCLCPP_WARN(this->get_logger(), "Set height %d failed!", param.as_int());
            }
        }
    }
    return result;
}
int main(int argc, char *argv[])
{

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto video_capture_node = std::make_shared<VideoCaptureNode>(options);

    exec.add_node(video_capture_node);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
