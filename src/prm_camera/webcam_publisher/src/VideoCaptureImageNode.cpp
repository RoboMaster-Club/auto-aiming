#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{

    int fps;
    int width;
    int height;
    bool native_size;
    int exposure;
    std::string source;
    std::string topic;
    std::string frame_id;

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);

    node->declare_parameter("fps", 30);
    node->declare_parameter("width", 1280);
    node->declare_parameter("height", 720);
    node->declare_parameter("source", "/dev/video0");
    node->declare_parameter("topic", "camera/image");
    node->declare_parameter("frame_id", "camera");
    node->declare_parameter("native_size", true);
    node->declare_parameter("exposure", 0);

    node->get_parameter("fps", fps);
    node->get_parameter("width", width);
    node->get_parameter("height", height);
    node->get_parameter("source", source);
    node->get_parameter("topic", topic);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("native_size", native_size);
    node->get_parameter("exposure", exposure);

    image_transport::ImageTransport it(node);


    RCLCPP_INFO_ONCE(node->get_logger(), "Publishing %s on %s", source.c_str(), topic.c_str());

    image_transport::Publisher pub = it.advertise(topic, 1);

    cv::Mat frame;
    cv::VideoCapture cap(source);
    std::cout << "source: " << source << std::endl;
    if (!cap.isOpened()) // check if open succeeded
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open video source");
        rclcpp::shutdown();
    }
    if (source.compare("/dev/video0") == 0)
    {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }

    if (fps > 0)
    {
        cap.set(cv::CAP_PROP_FPS, fps);
    }

    if (!native_size)
    {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        // cap.set(cv::CAP_PROP_EXPOSURE, 50);
    }

    if (exposure != 0)
    {
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        cap.set(cv::CAP_PROP_EXPOSURE, exposure);
        cap.set(cv::CAP_PROP_GAIN, 0);
        RCLCPP_INFO_ONCE(node->get_logger(), "Set exposure to %d", exposure);
    }

    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    uint32_t seq = 0;

    rclcpp::WallRate loop_rate(fps + 1);
    do
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
        cap.read(frame);
        if (fps == 0)
        {
            cv::imshow("image", frame);
            cv::waitKey(0);
        }
        if (frame.empty())
        {
            break;
        }
        hdr.stamp = node->now();

        hdr.frame_id = frame_id + std::to_string(seq);
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
        // cv::imshow("image", frame);
        // cv::waitKey(1);
        pub.publish(msg);
        RCLCPP_INFO_ONCE(node->get_logger(), "First Frame Published");

        seq++;
        // printf("seq: %d\n", seq);
        if (seq % 1000 == 0)
            RCLCPP_INFO (node->get_logger(), "seq: %d", seq);

    } while (rclcpp::ok());
    RCLCPP_INFO(node->get_logger(), "Cap ended at %d", seq);
}
