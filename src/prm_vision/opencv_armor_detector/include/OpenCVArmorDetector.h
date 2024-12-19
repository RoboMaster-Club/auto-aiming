#ifndef _OPENCVARMORDETECTOR_HPP
#define _OPENCVARMORDETECTOR_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

// Detector Constants
#define LIGHT_BAR_ANGLE_LIMIT 25.0
#define LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT 1.0
#define LIGHT_BAR_WIDTH_LOWER_LIMIT 2.0
#define LIGHT_BAR_HEIGHT_LOWER_LIMIT 5.0
#define ARMOR_ANGLE_DIFF_LIMIT 10.0
#define ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT 5.0
#define ARMOR_Y_DIFF_LIMIT 1.5
#define ARMOR_HEIGHT_RATIO_LIMIT 1.5
#define ARMOR_ASPECT_RATIO_LIMIT 5.0

#define WIDTH 1280
#define HEIGHT 1024

enum TargetColor
{
  BLUE,
  RED
};

struct DetectorConfig
{
  TargetColor _target_color;
  int _hue_range_limit;
  int _saturation_lower_limit;
  int _value_lower_limit;
  int _max_missed_frames;
};

class OpenCVArmorDetector
{
public:
  OpenCVArmorDetector(const DetectorConfig &config) : _config(config) { setConfig(config); }
  ~OpenCVArmorDetector() {}

  // Class methods
  std::vector<_Float32> search(cv::Mat &frame);

  // Other class variables
  int _detected_frame = 0;
  int _frame_count = 0;

  // Setters
  void setConfig(DetectorConfig config);

private:
  // Class methods
  std::vector<cv::Point2f> detectArmorsInFrame(cv::Mat &frame);

  // Class variables
  DetectorConfig _config;
  int _search_area[4] = {0, 0, WIDTH, HEIGHT};
  bool _reset_search_area = true;
  int _missed_frames = 0;

  // Dynamic parameters
  TargetColor _targetColor;
  int _max_missed_frames;

  // Helpers
  bool isLightBar(cv::RotatedRect &rect);
  bool isArmor(cv::RotatedRect &left_rect, cv::RotatedRect &right_rect);
  std::vector<cv::Point2f> rectToPoint(cv::RotatedRect &rect);
  void drawRotatedRect(cv::Mat &frame, cv::RotatedRect &rect);

  // Color limits
  cv::Scalar _blue_lower_limit;
  cv::Scalar _blue_upper_limit;
  cv::Scalar _red_lower_limit_1;
  cv::Scalar _red_upper_limit_1;
  cv::Scalar _red_lower_limit_2;
  cv::Scalar _red_upper_limit_2;
};
#endif //_OPENCVARMORDETECTOR_HPP