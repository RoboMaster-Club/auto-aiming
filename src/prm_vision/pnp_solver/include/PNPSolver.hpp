#ifndef _PNP_SOLVER_HPP
#define _PNP_SOLVER_HPP

#include <fstream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/key_points.hpp"
#include "vision_msgs/msg/predicted_armor.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "projectile_angle_convel.hpp"

#include "KalmanFilter.hpp"
#include "ValidityFilter.hpp"

#define PI 3.141592653589793238462643383

#define X_OFFSET 0 // Right of sensor
#define Y_OFFSET 0 // Down of sensor
#define Z_OFFSET 0 // Out of sensor

#define LIGHTBAR_HALF_HEIGHT 54.f / 2.f
#define SMALL_ARMOR_HALF_WIDTH 134.f / 2.f
#define LARGE_ARMOR_HALF_WIDTH 225.f / 2.f


using namespace std::chrono_literals;

class PNPSolver : public rclcpp::Node
{
public:
    PNPSolver(const rclcpp::NodeOptions &);
    ~PNPSolver();

private:
    std::vector<cv::Point3f> small_armor_object_points;
    std::vector<cv::Point3f> large_armor_object_points;
    cv::Mat distortion_coefficient;
    cv::Mat camera_matrix;
    float camera_matrix_[3][3];

    cv::Mat rvec;
    cv::Mat tvec;

    rclcpp::Subscription<vision_msgs::msg::KeyPoints>::SharedPtr key_points_subscriber;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::PredictedArmor>> predicted_armor_publisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> auto_aim_tracking_status_publisher = NULL;

    uint32_t seq_;

    KalmanFilter kalman_filter_;
    ValidityFilter validity_filter_;

    rclcpp::Time prev_time_;

    bool publish_tf_;
    float dst_[6];

    int locked_in_frames = 0;
    int num_frames_to_fire_after = 10; // corresponds to approximately 2/3 of a second
    double last_fire_time = 0;

    rclcpp::TimerBase::SharedPtr firing_timer;
    void keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr);
    void publishZeroPredictedArmor(std_msgs::msg::Header header);
    void check_last_firing_time();
};

#endif // _PNP_SOLVER_HPP
