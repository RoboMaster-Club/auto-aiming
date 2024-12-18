#ifndef VIDEO_CAPTURE_NODE_HPP
#define VIDEO_CAPTURE_NODE_HPP

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

    class VideoCaptureNode : public rclcpp::Node
    {
    public:
        explicit VideoCaptureNode(const rclcpp::NodeOptions &);
        ~VideoCaptureNode() override;

    private:
        void declare_parameters();

        void capture_frame();

        rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &);

        cv::VideoCapture cap;

        int fps;
        int width;
        int height;
        bool native_size;
        int exposure;
        std::string source;
        std::string frame_id;

        uint32_t seq;
        cv::Mat frame;

        std_msgs::msg::Header sensor_msg_header;
        sensor_msgs::msg::Image::SharedPtr image_msg;

        // sensor_msgs::msg::Image image_msg_;

        std::string camera_name_;
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
        sensor_msgs::msg::CameraInfo camera_info_msg;
        image_transport::CameraPublisher camera_pub;

        std::thread capture_thread;

        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

        // rclcpp::TimerBase::SharedPtr timer_;
    };

#endif // VIDEO_CAPTURE_NODE_HPP