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
    if (key_points_msg->points.size() != 8 || key_points_msg->points[4] == 0)
    {
        // No armor detected
        publishZeroPredictedArmor(key_points_msg->header);
        return;
    }

    // Convert the message to a vector of cv::Point2f
    std::vector<cv::Point2f> image_points;
    image_points.push_back(cv::Point2f(key_points_msg->points[0], key_points_msg->points[1]));
    image_points.push_back(cv::Point2f(key_points_msg->points[2], key_points_msg->points[3]));
    image_points.push_back(cv::Point2f(key_points_msg->points[4], key_points_msg->points[5]));
    image_points.push_back(cv::Point2f(key_points_msg->points[6], key_points_msg->points[7]));

    // Compute armor's pose and validate it
    cv::Mat tvec, rvec;
    pose_estimator->estimateTranslation(image_points, key_points_msg->is_large_armor, tvec, rvec);
    std::string auto_aim_status;
    bool valid = pose_estimator->isValid(tvec.at<float>(0), tvec.at<float>(1), tvec.at<float>(2), auto_aim_status);

    // Publish the predicted armor if it is valid
}

void PoseEstimatorNode::publishZeroPredictedArmor(std_msgs::msg::Header header)
{
    vision_msgs::msg::PredictedArmor predicted_armor_msg;
    predicted_armor_msg.header = header;
    predicted_armor_msg.x = 0.0;
    predicted_armor_msg.y = 0.0;
    predicted_armor_msg.z = 0.0;
    predicted_armor_msg.rvec_x = 0;
    predicted_armor_msg.rvec_y = 0;
    predicted_armor_msg.rvec_z = 0;
    predicted_armor_msg.x_vel = 0;
    predicted_armor_msg.y_vel = 0;
    predicted_armor_msg.z_vel = 0;

    // We may still fire for a short time after losing the target
    predicted_armor_msg.fire = false;
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