#include "PoseEstimatorNode.hpp"

PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions &options) : Node("pose_estimator", options)
{
    RCLCPP_INFO(get_logger(), "PoseEstimatorNode has been started.");

    // Initialize the pose estimator
    pose_estimator = new PoseEstimator();

    // Callbacks and pub/sub
    key_points_subscriber = this->create_subscription<vision_msgs::msg::KeyPoints>("key_points", 10, std::bind(&PoseEstimatorNode::keyPointsCallback, this, std::placeholders::_1));
    predicted_armor_publisher = this->create_publisher<vision_msgs::msg::PredictedArmor>("predicted_armor", 10);
}

PoseEstimatorNode::~PoseEstimatorNode() { delete pose_estimator; }

void PoseEstimatorNode::keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr key_points_msg)
{
    if (key_points_msg->points.size() != 8 || key_points_msg->points[4] == 0) // idx 4 is x-coord of top right corner, so if it's 0, we know there's no armor
    {
        // No armor detected
        publishZeroPredictedArmor(key_points_msg->header);
        return;
    }

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
        _allowed_missed_frames_before_no_fire = 5;

        vision_msgs::msg::PredictedArmor predicted_armor_msg;
        predicted_armor_msg.header = key_points_msg->header;
        predicted_armor_msg.x = tvec.at<float>(0);
        predicted_armor_msg.y = tvec.at<float>(1);
        predicted_armor_msg.z = tvec.at<float>(2);
        predicted_armor_msg.pitch = 15 * CV_PI / 180; // 15 degrees in radians
        predicted_armor_msg.yaw = _last_yaw_estimate; // Use the latest yaw estimate
        predicted_armor_msg.roll = 0;                 // Roll is always 0
        predicted_armor_msg.x_vel = 0;
        predicted_armor_msg.y_vel = 0; // TODO: compute yaw rate
        predicted_armor_msg.z_vel = 0;
        predicted_armor_msg.fire = (new_auto_aim_status == "FIRE");
        predicted_armor_publisher->publish(predicted_armor_msg);
    }
    else
    {
        publishZeroPredictedArmor(key_points_msg->header);
    }
}

void PoseEstimatorNode::publishZeroPredictedArmor(std_msgs::msg::Header header)
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
    predicted_armor_msg.fire = (_allowed_missed_frames_before_no_fire-- > 0);

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