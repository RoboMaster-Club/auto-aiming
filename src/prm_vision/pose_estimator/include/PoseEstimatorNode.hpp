#ifndef POSE_ESTIMATOR_NODE_HPP
#define POSE_ESTIMATOR_NODE_HPP

// OpenCV and ROS2 includes
#include <opencv2/opencv.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"
#include "vision_msgs/msg/key_points.hpp"
#include "vision_msgs/msg/key_point_groups.hpp"
#include "vision_msgs/msg/predicted_armor.hpp"

// tf publishing
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Pose Estimator classes
#include "PoseEstimator.h"

#include "AntiSpintop.hpp"

class PoseEstimatorNode : public rclcpp::Node
{
public:
    PoseEstimatorNode(const rclcpp::NodeOptions &options);
    ~PoseEstimatorNode();

    PoseEstimator *pose_estimator = new PoseEstimator();
    ValidityFilter &validity_filter_ = pose_estimator->validity_filter_; // Reference to the validity filter

private:
    double _last_yaw_estimate = 0.0;
    double last_yaw_estimates[2] = {0.0, 0.0};

    AntiSpintop* anti_spintop;

    // Class methods
    void publishZeroPredictedArmor(std_msgs::msg::Header header, std::string new_auto_aim_status);

    // Callbacks and publishers/subscribers
    // rclcpp::Subscription<vision_msgs::msg::KeyPoints>::SharedPtr key_points_subscriber;
    rclcpp::Subscription<vision_msgs::msg::KeyPointGroups>::SharedPtr keypoint_groups_subscriber;

    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::PredictedArmor>> predicted_armor_publisher;
    void keyPointsCallback(const vision_msgs::msg::KeyPointGroups::SharedPtr keypoint_group_msg);
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
};

#endif // POSE_ESTIMATOR_NODE_HPP