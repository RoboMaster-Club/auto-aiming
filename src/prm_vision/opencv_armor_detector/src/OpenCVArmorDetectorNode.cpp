#include "OpenCVArmorDetectorNode.hpp"

OpenCVArmorDetectorNode::OpenCVArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("opencv_armor_detector", options)
{
  RCLCPP_INFO(get_logger(), "OpenCVArmorDetectorNode has been started.");

  // These are dynamic ROS2 parameters, meaning they can be changed during runtime for testing purposes.
  _hue_range_limit = this->declare_parameter("_hue_range_limit", 30);
  _saturation_lower_limit = this->declare_parameter("_saturation_lower_limit", 100);
  _value_lower_limit = this->declare_parameter("_value_lower_limit", 150);
  _target_color = this->declare_parameter("_target_red", true) ? RED : BLUE;
  _max_missed_frames = this->declare_parameter("_max_missed_frames", 1);
  _reduce_search_area = this->declare_parameter("_reduce_search_area", true);
  have_last_target_ = false;
  // Callbacks and pub/sub
  params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&OpenCVArmorDetectorNode::parameters_callback, this, std::placeholders::_1));
  keypoints_publisher = this->create_publisher<vision_msgs::msg::KeyPoints>("key_points", 10);

  // Initialize the detector
  DetectorConfig config = {_target_color, _hue_range_limit, _saturation_lower_limit, _value_lower_limit, _max_missed_frames, _reduce_search_area};
  detector = new OpenCVArmorDetector(config);
}

OpenCVArmorDetectorNode::~OpenCVArmorDetectorNode() { delete detector; }

void OpenCVArmorDetectorNode::imageTransportInitilization()
{
  image_transport::ImageTransport it(this->shared_from_this());
  image_subscriber =
      it.subscribe("image_raw", 1,
                   std::bind(&OpenCVArmorDetectorNode::imageCallback, this,
                             std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult OpenCVArmorDetectorNode::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &parameter : parameters)
  {
    if (parameter.get_name() == "_hue_range_limit")
    {
      _hue_range_limit = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New hue range limit: %d", _hue_range_limit);
    }
    else if (parameter.get_name() == "_saturation_lower_limit")
    {
      _saturation_lower_limit = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New saturation lower limit: %d",
                  _saturation_lower_limit);
    }
    else if (parameter.get_name() == "_value_lower_limit")
    {
      _value_lower_limit = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New value lower limit: %d",
                  _value_lower_limit);
    }
    else if (parameter.get_name() == "_target_red")
    {
      _target_color = parameter.as_bool() ? RED : BLUE;
      RCLCPP_INFO(get_logger(), "New target color: %s",
                  _target_color ? "red" : "blue");
    }
    else if (parameter.get_name() == "_max_missed_frames")
    {
      _max_missed_frames = parameter.as_int();
      RCLCPP_INFO(get_logger(), "New max missed frames: %d",
                  _max_missed_frames);
    }
    else if (parameter.get_name() == "_reduce_search_area")
    {
      _reduce_search_area = parameter.as_bool();
      RCLCPP_INFO(get_logger(), "New reduce search area: %s",
                  _reduce_search_area ? "true" : "false");
    }
    else
    {
      result.successful = false;
      return result;
    }
  }

  // Update the detector's config
  detector->setConfig({_target_color, _hue_range_limit, _saturation_lower_limit, _value_lower_limit, _max_missed_frames, _reduce_search_area});
  return result;
}

void OpenCVArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
    cv::Mat full = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    cv::resize(full, full, cv::Size(WIDTH, HEIGHT));

    cv::Rect roi(0, 0, WIDTH, HEIGHT);
    if (have_last_target_) {
        int dx = last_target_roi_.width  / 5;
        int dy = last_target_roi_.height / 5;
        roi = (last_target_roi_ + cv::Size(2*dx, 2*dy) - cv::Point(dx, dy))
              & cv::Rect(0, 0, WIDTH, HEIGHT);
    }

    cv::Mat crop = full(roi).clone();
    auto points = detector->search(crop);

    bool found = std::any_of(points.begin(), points.end(),
                             [](float v){ return v != 0.0f; });
    if (!found && have_last_target_) {
        have_last_target_ = false;
        points = detector->search(full);
        found = std::any_of(points.begin(), points.end(),
                            [](float v){ return v != 0.0f; });
    }
    if (!found) {
        return;
    }

    for (int i = 0; i < 4; ++i) {
        points[2*i + 0] += float(roi.x);
        points[2*i + 1] += float(roi.y);
    }

    int xmin = std::min({points[0], points[2], points[4], points[6]});
    int xmax = std::max({points[0], points[2], points[4], points[6]});
    int ymin = std::min({points[1], points[3], points[5], points[7]});
    int ymax = std::max({points[1], points[3], points[5], points[7]});
    last_target_roi_   = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
    have_last_target_ = true;

    vision_msgs::msg::KeyPoints keypoints_msg;
    std::array<float, 8> points_array;
    std::copy(points.begin(), points.end(), points_array.begin());
    keypoints_msg.header         = image_msg->header;
    keypoints_msg.points         = points_array;
    float h = std::min(
        cv::norm(cv::Point2f(points[1],points[1]) - cv::Point2f(points[3],points[3])),
        cv::norm(cv::Point2f(points[5],points[5]) - cv::Point2f(points[7],points[7]))
    );
    float w = cv::norm(
        (cv::Point2f(points[0],points[0]) + cv::Point2f(points[2],points[2]))*0.5f
        - (cv::Point2f(points[4],points[4]) + cv::Point2f(points[6],points[6]))*0.5f
    );
    keypoints_msg.is_large_armor = (w / h) > 3.0f;

    keypoints_publisher->publish(keypoints_msg);
}






int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto opencv_armor_detector_node =
      std::make_shared<OpenCVArmorDetectorNode>(options);
  opencv_armor_detector_node->imageTransportInitilization();

  exec.add_node(opencv_armor_detector_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
