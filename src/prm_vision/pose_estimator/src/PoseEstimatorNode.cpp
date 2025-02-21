#include "PoseEstimatorNode.hpp"

PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions &options) : Node("pose_estimator", options)
{
    RCLCPP_INFO(get_logger(), "PoseEstimatorNode has been started.");

    // Callbacks and pub/sub
    key_points_subscriber = this->create_subscription<vision_msgs::msg::KeyPoints>("key_points", 10, std::bind(&PoseEstimatorNode::keyPointsCallback, this, std::placeholders::_1));
    predicted_armor_publisher = this->create_publisher<vision_msgs::msg::PredictedArmor>("predicted_armor", 10);

    // Dynamic parameters
    pose_estimator->setAllowedMissedFramesBeforeNoFire(this->declare_parameter("_allowed_missed_frames_before_no_fire", 150));
    pose_estimator->setNumFramesToFireAfter(this->declare_parameter("_num_frames_to_fire_after", 3));
    pose_estimator->setEpsilon(this->declare_parameter("_epsilon", 0.4));
    pose_estimator->setMaxIterations(this->declare_parameter("_max_iterations", 50));
    pose_estimator->setPitch(this->declare_parameter("_pitch", 15.0));
    
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
        else if (param.get_name() == "_max_dt" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            validity_filter_.setMaxDt(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_dt' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "_allowed_missed_frames_before_no_fire" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            pose_estimator->setAllowedMissedFramesBeforeNoFire(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_allowed_missed_frames_before_no_fire' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_epsilon" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            pose_estimator->setEpsilon(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter '_epsilon' updated to: %f", param.as_double());
        }
        else if (param.get_name() == "_max_iterations" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            pose_estimator->setMaxIterations(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Parameter '_max_iterations' updated to: %d", param.as_int());
        }
        else if (param.get_name() == "_pitch" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            pose_estimator->setPitch(param.as_double());
            RCLCPP_INFO(this->get_logger(), "Parameter '_pitch' updated to: %f", param.as_double());
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

void PoseEstimatorNode::keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr key_points_msg)
{
    if (key_points_msg->points.size() != 8 || key_points_msg->points[4] == 0) // idx 4 is x-coord of top right corner, so if it's 0, we know there's no armor
    {
        // No armor detected
        publishZeroPredictedArmor(key_points_msg->header, "NO_ARMOR");
        return;
    }

    static int ctr = 0;
    ctr++;

    cv::Mat tvec, rvec;
    bool reset_kalman = false;
    std::string new_auto_aim_status;
    std::vector<cv::Point2f> image_points;

    // Convert the message to a vector of cv::Point2f
    image_points.push_back(cv::Point2f(key_points_msg->points[0], key_points_msg->points[1]));
    image_points.push_back(cv::Point2f(key_points_msg->points[2], key_points_msg->points[3]));
    image_points.push_back(cv::Point2f(key_points_msg->points[4], key_points_msg->points[5]));
    image_points.push_back(cv::Point2f(key_points_msg->points[6], key_points_msg->points[7]));

    // Compute armor's pose and validate it. We compute new yaw estimate based on the last yaw estimate.
    pose_estimator->estimateTranslation(image_points, key_points_msg->is_large_armor, tvec, rvec);
    _last_yaw_estimate = pose_estimator->estimateYaw(_last_yaw_estimate, image_points, tvec);
    bool valid_pose_estimate = pose_estimator->isValid(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), new_auto_aim_status, reset_kalman);

    // Publish the predicted armor if the pose is valid (we are tracking or firing)
    if (valid_pose_estimate)
    {
        vision_msgs::msg::PredictedArmor predicted_armor_msg;
        predicted_armor_msg.header = key_points_msg->header;
        predicted_armor_msg.x = tvec.at<double>(0);
        predicted_armor_msg.y = tvec.at<double>(1);
        predicted_armor_msg.z = tvec.at<double>(2);
        predicted_armor_msg.pitch = 15;                             // Pitch is always 15
        predicted_armor_msg.yaw = 180 * _last_yaw_estimate / CV_PI; // Convert to degrees
        predicted_armor_msg.roll = 0;                               // Roll is always 0
        predicted_armor_msg.x_vel = 0;
        predicted_armor_msg.y_vel = 0; // TODO: compute yaw rate
        predicted_armor_msg.z_vel = 0;
        predicted_armor_msg.fire = (new_auto_aim_status == "FIRE");
        predicted_armor_publisher->publish(predicted_armor_msg);
    }

    else if (new_auto_aim_status == "STOPPING")
    {
        publishZeroPredictedArmor(key_points_msg->header, new_auto_aim_status);
    }

    // Draw top-down view
#ifdef DEBUG
    if (ctr % 100 == 0) {
        drawTopDownViewGivenRotation(_last_yaw_estimate, tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    }
#endif
}

void PoseEstimatorNode::drawTopDownViewGivenRotation(double yaw, double X, double Y, double Z)
{
    // Define image dimensions
    const int imageWidth = 800;   // Width of the top-down view image
    const int imageHeight = 1000; // Height of the top-down view image

    // Create a blank image (white background)
    cv::Mat topDownImage(imageHeight, imageWidth, CV_8UC3, cv::Scalar(255, 255, 255));

    // Define scaling factor (pixels per millimeter)
    // Since Z ranges from 500 mm to 4000 mm, we scale to fit within the image height
    double scale = imageHeight / 4000.0; // Scale to fit 4000 mm (4 meters) in the image height

    // Map real-world coordinates (X, Z) to image pixel coordinates
    int centerX = static_cast<int>(imageWidth / 2 + X * scale); // Center X in image
    int centerZ = static_cast<int>(imageHeight - Z * scale);    // Center Z in image (flip Y-axis for top-down view)

    // Define line length (adjustable)
    double lineLength = 20.0; // Length of the line in pixels
    double robot_diameter = 150.0;

    // Calculate line endpoints based on yaw angle
    double angleRad = -yaw; // Convert yaw to radians

    // draw armor
    drawLineWithAngle(topDownImage, cv::Point2f(centerX, centerZ), angleRad, lineLength, cv::Scalar(0, 0, 255));
    drawLineWithAngle(topDownImage, cv::Point2f(centerX, centerZ), angleRad + CV_PI, lineLength, cv::Scalar(0, 0, 255));

    // draw line of typical robot diameter
    drawLineWithAngle(topDownImage, cv::Point2f(centerX, centerZ), angleRad - CV_PI / 2, robot_diameter, cv::Scalar(0, 255, 0));

    // armor at other end of line
    drawLineWithAngle(topDownImage, cv::Point2f(centerX + robot_diameter * cos(angleRad - CV_PI / 2), centerZ + robot_diameter * sin(angleRad - CV_PI / 2)), angleRad, lineLength, cv::Scalar(0, 0, 255));
    drawLineWithAngle(topDownImage, cv::Point2f(centerX + robot_diameter * cos(angleRad - CV_PI / 2), centerZ + robot_diameter * sin(angleRad - CV_PI / 2)), angleRad + CV_PI, lineLength, cv::Scalar(0, 0, 255));

    // more lines of typical robot diameter 90 degrees from the center line
    cv::Point2f robot_center = cv::Point2f((centerX + robot_diameter / 2 * cos(angleRad - CV_PI / 2)), (centerZ + robot_diameter / 2 * sin(angleRad - CV_PI / 2)));
    drawLineWithAngle(topDownImage, robot_center, angleRad - CV_PI, robot_diameter / 2, cv::Scalar(200, 255, 0));
    drawLineWithAngle(topDownImage, robot_center, angleRad, robot_diameter / 2, cv::Scalar(200, 255, 0));

    // armor at other end of these two lines, perpendicular to the center line
    drawLineWithAngle(topDownImage, cv::Point2f(robot_center.x + robot_diameter / 2 * cos(angleRad), robot_center.y + robot_diameter / 2 * sin(angleRad)), angleRad - CV_PI / 2, lineLength, cv::Scalar(0, 0, 255));
    drawLineWithAngle(topDownImage, cv::Point2f(robot_center.x + robot_diameter / 2 * cos(angleRad), robot_center.y + robot_diameter / 2 * sin(angleRad)), angleRad + CV_PI / 2, lineLength, cv::Scalar(0, 0, 255));

    // same thing on the other side
    drawLineWithAngle(topDownImage, cv::Point2f(robot_center.x - robot_diameter / 2 * cos(angleRad), robot_center.y - robot_diameter / 2 * sin(angleRad)), angleRad - CV_PI / 2, lineLength, cv::Scalar(0, 0, 255));
    drawLineWithAngle(topDownImage, cv::Point2f(robot_center.x - robot_diameter / 2 * cos(angleRad), robot_center.y - robot_diameter / 2 * sin(angleRad)), angleRad + CV_PI / 2, lineLength, cv::Scalar(0, 0, 255));

    RCLCPP_INFO(get_logger(), "Drawing top-down view with yaw = %.2f, X = %.2f, Y = %.2f, Z = %.2f", 180.0 * yaw / CV_PI, X, Y, Z);

    // Display the image
    cv::imshow("Top-Down View", topDownImage);
    cv::waitKey(1);
}

void PoseEstimatorNode::drawLineWithAngle(cv::Mat &image, cv::Point2f start_pt, double angle, double length, cv::Scalar color)
{
    // Calculate endpoint of the line
    cv::Point2f end_pt;
    end_pt.x = start_pt.x + length * cos(angle);
    end_pt.y = start_pt.y + length * sin(angle);

    // Draw the line
    cv::line(image, start_pt, end_pt, color, 2);
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