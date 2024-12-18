#include <cstdio>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
// #include "recorder.hpp"

class VideoRecorderNode : public rclcpp::Node
{
public:
  VideoRecorderNode()
      : Node("video_recorder"), img_transport_(std::shared_ptr<VideoRecorderNode>(this, [](VideoRecorderNode *) {}))
  {
    // Define input topic to get camera feed
    std::string input_topic = this->declare_parameter("topic", "/image_raw");
    dst_ = this->declare_parameter("dst", "out");
    fps_ = this->declare_parameter("fps", 30);
    frame_count_ = this->declare_parameter("frame_count", 0);

    height_ = this->declare_parameter("height", 720);
    width_ = this->declare_parameter("width", 1289);

    to_image_ = this->declare_parameter("to_image", 0);

    size_ = cv::Size(width_, height_);

    // Subscribe to input image topic and print message
    image_sub_ = img_transport_.subscribe(input_topic, 1, &VideoRecorderNode::imageCallback, this);
    resized_frame_ = cv::Mat(cv::Size(height_, width_), CV_8UC3);

    RCLCPP_INFO(this->get_logger(), "Subscribed to input topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Out put to file: %s", dst_.c_str());
    RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_);
    RCLCPP_INFO(this->get_logger(), "Width: %d, Height: %d", width_, height_);
    RCLCPP_INFO(this->get_logger(), "Recording frame count: %d", frame_count_);
  }

private:
  cv::VideoWriter writer_;
  int fps_;
  int height_, width_;
  std::string dst_;
  cv::Mat resized_frame_;

  int to_image_;

  cv::Size size_;
  uint32_t seq_, frame_count_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {

    // Print a message to indicate that an image has been received

    RCLCPP_INFO_ONCE(this->get_logger(), "First frame recived.");

    // Convert ROS image message to OpenCV Mat
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (!to_image_) // To Video (lossy)
    {
      // Initialize the VideoWriter object if not already initialized
      if (!writer_.isOpened())
      {
        writer_.open(dst_ + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, size_);
      }

      // Write frame to video file
      if (frame.cols != height_ || frame.rows != width_)
      {
        cv::resize(frame, resized_frame_, size_);
        writer_.write(resized_frame_);
      }
      else
      {
        writer_.write(frame);
      }
    } else { // To Video

      cv::imwrite(dst_ + std::to_string(seq_) + ".png", frame);
    }

    if (seq_ % 100 == 0)
    {
      RCLCPP_INFO(this->get_logger(), "seq: %d", seq_);
    }
    seq_++;
    if (frame_count_ != 0 && seq_ >= frame_count_)
    {
      rclcpp::shutdown();
    }
  }

  image_transport::Subscriber image_sub_;
  image_transport::Publisher video_pub_;
  image_transport::ImageTransport img_transport_;
};

// int VideoRecorderNode::codec_;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
