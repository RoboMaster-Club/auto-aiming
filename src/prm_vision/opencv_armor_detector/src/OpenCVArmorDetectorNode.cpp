// ========================
// Code 2: Backend Uses Libraries
// ========================

#include "OpenCVArmorDetectorNode.hpp"

OpenCVArmorDetectorNode::OpenCVArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("opencv_armor_detector", options)
{
  RCLCPP_INFO(get_logger(), "OpenCVArmorDetectorNode has been started.");

  _hue_range_limit = this->declare_parameter("_hue_range_limit", 30);
  _saturation_lower_limit = this->declare_parameter("_saturation_lower_limit", 100);
  _value_lower_limit = this->declare_parameter("_value_lower_limit", 150);
  _target_color = this->declare_parameter("_target_red", true) ? RED : BLUE;
  _max_missed_frames = this->declare_parameter("_max_missed_frames", 1);
  _reduce_search_area = this->declare_parameter("_reduce_search_area", true);

  params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&OpenCVArmorDetectorNode::parameters_callback, this, std::placeholders::_1));
  keypoints_publisher = this->create_publisher<vision_msgs::msg::KeyPoints>("key_points", 10);

  DetectorConfig config = {_target_color, _hue_range_limit, _saturation_lower_limit, _value_lower_limit, _max_missed_frames, _reduce_search_area};
  detector = new OpenCVArmorDetector(config);
}

OpenCVArmorDetectorNode::~OpenCVArmorDetectorNode() { delete detector; }

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  const rclcpp::NodeOptions options;
  auto opencv_armor_detector_node =
      std::make_shared<OpenCVArmorDetectorNode>(options);
  exec.add_node(opencv_armor_detector_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
