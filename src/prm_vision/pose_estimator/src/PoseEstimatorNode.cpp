#include "PoseEstimatorNode.hpp"

PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions &options) : Node("pose_estimator", options)
{
    RCLCPP_INFO(get_logger(), "PoseEstimatorNode has been started.");

    anti_spintop = new AntiSpintop(10, 0.1);

    // Callbacks and pub/sub
    // key_points_subscriber = this->create_subscription<vision_msgs::msg::KeyPoints>("key_points", 10, std::bind(&PoseEstimatorNode::keyPointsCallback, this, std::placeholders::_1));
    keypoint_groups_subscriber = this->create_subscription<vision_msgs::msg::KeyPointGroups>(
        "key_points", 10, std::bind(&PoseEstimatorNode::keyPointsCallback, this, std::placeholders::_1));

    predicted_armor_publisher = this->create_publisher<vision_msgs::msg::PredictedArmor>("predicted_armor", 10);

    // Dynamic parameters
    pose_estimator->setAllowedMissedFramesBeforeNoFire(this->declare_parameter("_allowed_missed_frames_before_no_fire", 15));
    pose_estimator->setNumFramesToFireAfter(this->declare_parameter("_num_frames_to_fire_after", 3));
    validity_filter_.setLockInAfter(this->declare_parameter("_lock_in_after", 3));
    validity_filter_.setMaxDistance(this->declare_parameter("_max_distance", 10000));
    validity_filter_.setMinDistance(this->declare_parameter("_min_distance", 10));
    validity_filter_.setMaxShiftDistance(this->declare_parameter("_max_shift_distance", 150));
    validity_filter_.setPrevLen(this->declare_parameter("_prev_len", 5));
    validity_filter_.setMaxDt(this->declare_parameter("_max_dt", 2000.0));
    params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PoseEstimatorNode::parameters_callback, this, std::placeholders::_1));
}

PoseEstimatorNode::~PoseEstimatorNode() { delete pose_estimator; }

rcl_interfaces::msg::SetParametersResult PoseEstimatorNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters successfully updated.";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "_num_frames_to_fire_after" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            pose_estimator->setNumFramesToFireAfter(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_num_frames_to_fire_after' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_lock_in_after" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setLockInAfter(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_lock_in_after' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_max_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setMaxDistance(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_distance' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_min_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setMinDistance(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_min_distance' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_max_shift_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setMaxShiftDistance(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_shift_distance' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_prev_len" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            validity_filter_.setPrevLen(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_prev_len' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_max_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE || param.get_type())
        {
            validity_filter_.setMaxDt(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_dt' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "_allowed_missed_frames_before_no_fire" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            pose_estimator->setAllowedMissedFramesBeforeNoFire(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_allowed_missed_frames_before_no_fire' updated to: %d", param.as_int());
        }
        else
        {
            result.successful = false;
            result.reason = "Invalid parameter or type.";
            RCLCPP_WARN(this->get_logger(), "Failed to update parameter: %s", param.get_name().c_str());
        }
    }

    return result;
}

void PoseEstimatorNode::keyPointsCallback(const vision_msgs::msg::KeyPointGroups::SharedPtr keypoint_group_msg)
{
    vision_msgs::msg::KeyPoints armors[2];
    for (int index = 0; index < 2; index++) {
        armors[index] = keypoint_group_msg->groups[index];
    }

    // if no armor detected
    if (armors[0].points[0] == 0)
    {
        publishZeroPredictedArmor(armors[0].header, "NO_ARMOR");
        return;
    }

    cv::Mat tvecs[2];

    int spin_direction = anti_spintop->get_direction();

    for (int index = 0; index < keypoint_group_msg->num_armors; index++) {
        if (armors[index].points[0] == 0)
        {
            continue;
        }
        
        std::vector<cv::Point2f> image_points;
        cv::Mat tvec, rvec;

        image_points.push_back(cv::Point2f(armors[index].points[0], armors[index].points[1]));
        image_points.push_back(cv::Point2f(armors[index].points[2], armors[index].points[3]));
        image_points.push_back(cv::Point2f(armors[index].points[4], armors[index].points[5]));
        image_points.push_back(cv::Point2f(armors[index].points[6], armors[index].points[7]));

        // pose_estimator->estimateTranslation(image_points, keypoint_group_msg->groups[index].is_large_armor, tvec, rvec);
        
        // short yaw_estimate_idx = spin_direction == 1 ? keypoint_group_msg->num_armors - 1 - index : index;
        // last_yaw_estimates[index] = pose_estimator->estimateYaw(last_yaw_estimates[yaw_estimate_idx], image_points, tvec);

        // tvecs[index] = tvec;
    }


    cv::Mat aim_tvec;
    double aim_yaw;
    bool reset_kalman = false;
    std::string new_auto_aim_status;

    anti_spintop->calculate_aim_point(tvecs, last_yaw_estimates, keypoint_group_msg->num_armors, aim_tvec, aim_yaw);

    // TODO: fix segv
    // bool valid_pose_estimate = pose_estimator->isValid(aim_tvec.at<double>(0), aim_tvec.at<double>(1), aim_tvec.at<double>(2), new_auto_aim_status, reset_kalman);

    // // Publish the predicted armor if the pose is valid (we are tracking or firing)
    // if (valid_pose_estimate)
    // {
    //     vision_msgs::msg::PredictedArmor predicted_armor_msg;
    //     predicted_armor_msg.header = keypoint_group_msg->groups[0].header;
    //     predicted_armor_msg.x = aim_tvec.at<double>(0);
    //     predicted_armor_msg.y = aim_tvec.at<double>(1);
    //     predicted_armor_msg.z = aim_tvec.at<double>(2);
    //     predicted_armor_msg.pitch = 15;                             // Pitch is always 15
    //     predicted_armor_msg.yaw = 180 * aim_yaw / CV_PI;            // Convert to degrees
    //     predicted_armor_msg.roll = 0;                               // Roll is always 0
    //     predicted_armor_msg.x_vel = 0;
    //     predicted_armor_msg.y_vel = 0; // TODO: compute yaw rate
    //     predicted_armor_msg.z_vel = 0;
    //     predicted_armor_msg.fire = (new_auto_aim_status == "FIRE");
    //     predicted_armor_publisher->publish(predicted_armor_msg);
    // }

    // else if (new_auto_aim_status == "STOPPING")
    // {
    //     publishZeroPredictedArmor(keypoint_group_msg->groups[0].header, new_auto_aim_status);
    // }
}

void PoseEstimatorNode::publishZeroPredictedArmor(std_msgs::msg::Header header, std::string new_auto_aim_status)
{
    vision_msgs::msg::PredictedArmor predicted_armor_msg;
    predicted_armor_msg.header = header;
    predicted_armor_msg.x = 0;
    predicted_armor_msg.y = 0;
    predicted_armor_msg.z = 0;
    predicted_armor_msg.pitch = 0;
    predicted_armor_msg.yaw = 0;
    predicted_armor_msg.roll = 0;
    predicted_armor_msg.x_vel = 0;
    predicted_armor_msg.y_vel = 0;
    predicted_armor_msg.z_vel = 0;
    predicted_armor_msg.fire = (new_auto_aim_status == "FIRE");

    predicted_armor_publisher->publish(predicted_armor_msg);
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto pose_estimator_node = std::make_shared<PoseEstimatorNode>(options);

    exec.add_node(pose_estimator_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}