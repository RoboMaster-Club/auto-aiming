#include "PNPSolverNode.hpp"

PNPSolverNode::PNPSolverNode(const rclcpp::NodeOptions &options) : Node("pnp_solver", options)
{
    publish_tf_ = this->declare_parameter("publish_tf", false);
    publish_tf_ = true;
    RCLCPP_INFO(this->get_logger(), "Publish TF: %s", publish_tf_ ? "true" : "false");

    pnp_solver_ = PNPSolver();
    kalman_filter_ = KalmanFilter();
    validity_filter_ = ValidityFilter();

    seq_ = 0;

    prev_time_ = this->now();

    if (publish_tf_)
    {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // STOP FIRING if no target detected for 2 seconds
    this->firing_timer = this->create_wall_timer(500ms, std::bind(&PNPSolverNode::check_last_firing_time, this));

    // Initialize subscriber and publisher for predicted armor, key points, and auto aim tracking status
    predicted_armor_publisher = this->create_publisher<vision_msgs::msg::PredictedArmor>("predicted_armor", 10);
    auto_aim_tracking_status_publisher = this->create_publisher<std_msgs::msg::String>("auto_aim_tracking_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    key_points_subscriber = this->create_subscription<vision_msgs::msg::KeyPoints>(
        "key_points", 10, std::bind(&PNPSolverNode::keyPointsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PNP Solver Created");
}

PNPSolverNode::~PNPSolverNode()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Destorying PNP Solver Node");
    return;
}

/**
 * @brief Validates the translational coordinates, determines the status of auto aiming, and resets the
 * Kalman filter accordingly
 * 
 * @param predicted_armor_msg The reference of the predicted armor message object
 * @param header The header to be published
 * @param tvec The translatoinal coordinates
 */
void PNPSolverNode::validateCoordinates(vision_msgs::msg::PredictedArmor *predicted_armor_msg, std_msgs::msg::Header header, std::vector<float> tvec)
{
    std_msgs::msg::String auto_aim_tracking_status_msg;
    rclcpp::Time curr_time_(header.stamp.sec, header.stamp.nanosec, RCL_ROS_TIME);
    double dt = (curr_time_.seconds() - prev_time_.seconds());

    bool reset_kalman = validity_filter_.validation(tvec[0], tvec[1], tvec[2], dt);
    if (reset_kalman)
    {
        auto_aim_tracking_status_msg.data = "RESET KALMAN";
        auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        kalman_filter_.reset();
        RCLCPP_INFO(this->get_logger(), "Validity Filter Reset Kalman Filter");
        prev_time_ = curr_time_;

        predicted_armor_msg->x_vel = 0;
        predicted_armor_msg->y_vel = 0;
        predicted_armor_msg->z_vel = 0;
    }
    else
    {
        predicted_armor_msg->x_vel = dst_[0];
        predicted_armor_msg->y_vel = dst_[1];
        predicted_armor_msg->z_vel = dst_[2];
    }

    // Stopping motor
    if (validity_filter_.state == STOPPING)
    {
        auto_aim_tracking_status_msg.data = "STOPPING";
        auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        publishZeroPredictedArmor(header);
        locked_in_frames = 0;
        return;
    }

    // Idling, dont send anything
    else if (validity_filter_.state == IDLING)
    {
        auto_aim_tracking_status_msg.data = "IDLING";
        auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        locked_in_frames = 0;
        return;
    }

    // Tracking, update kalman filter
    else if (validity_filter_.state == TRACKING)
    {
        // if tracking and we have locked in for enough frames, fire at the target
        if (locked_in_frames > num_frames_to_fire_after)
        {
            auto_aim_tracking_status_msg.data = "FIRE";
            auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
            last_fire_time = this->now().seconds();
        }
        else
        {
            auto_aim_tracking_status_msg.data = "TRACKING";
            auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        }

        kalman_filter_.update(tvec[0], tvec[1], tvec[2], dt, dst_);
        prev_time_ = curr_time_;
        locked_in_frames++;
    }
}

/**
 * @brief Publishes the predicted armor as a ROS message
 * 
 * @param predicted_armor_msg The reference of the predicted armor message object
 * @param header The header to be published
 * @param tvec The translational coordinates
 * @param rvec The rotational coordinates
 */
void PNPSolverNode::publishPredictedArmor(vision_msgs::msg::PredictedArmor *predicted_armor_msg, std_msgs::msg::Header header, std::vector<float> tvec, std::vector<float> rvec)
{
    predicted_armor_msg->header = header;
    double distance = sqrt(tvec[0] * tvec[0] + tvec[1] * tvec[1] + tvec[2] * tvec[2]);
    predicted_armor_msg->x = tvec[0];
    predicted_armor_msg->y = tvec[1] + 50;
    predicted_armor_msg->z = tvec[2] - 60; // 50mm coordinate transform in Z     // + (0.1*distance + -127.75);

    vec3 P(tvec[0] / 1000, tvec[1] / 1000, tvec[2] / 1000), V(0, 0, 0), grav(0, 0, 9.81);
    double p = 0, y = 0;
    bool im = 0;
    pitch_yaw_gravity_model_movingtarget_const_v(P, V, grav, 0, &p, &y, &im);
    y = y * (tvec[1] > 0 ? -1 : 1); // currently a bug where yaw is never negative, so we just multiply by the sign of "y" of the target

    double yaw = -atan(tvec[1] / tvec[0]) * 180 / PI;
    double pitch = atan(tvec[2] / tvec[0]) * 180 / PI;

    // fire at the target
    predicted_armor_msg->fire = locked_in_frames > num_frames_to_fire_after;

    // Rotation XYZ
    // NOTE: Eventually this should be passed into the kalman filter and predicted_rvec should be published instead
    predicted_armor_msg->rvec_x = rvec[0];
    predicted_armor_msg->rvec_y = rvec[1];
    predicted_armor_msg->rvec_z = rvec[2];
    predicted_armor_publisher->publish(*predicted_armor_msg);
}

/**
 * @brief Publishes the transformation of detected, filtered, and predicted armor, as a ROS message
 * 
 * @param header The header to be published
 * @param tvec The translational coordinates
 */
void PNPSolverNode::publishTF(std_msgs::msg::Header header, std::vector<float> tvec)
{
    // Publish TF raw results
    geometry_msgs::msg::TransformStamped detected_armor_tf;
    detected_armor_tf.header.stamp = header.stamp;
    detected_armor_tf.header.frame_id = "camera_link";
    detected_armor_tf.child_frame_id = "detected_armor";
    detected_armor_tf.transform.translation.x = tvec[0] / 1000;
    detected_armor_tf.transform.translation.y = tvec[1] / 1000;
    detected_armor_tf.transform.translation.z = tvec[2] / 1000;
    detected_armor_tf.transform.rotation.x = 1;
    detected_armor_tf.transform.rotation.y = 0;
    detected_armor_tf.transform.rotation.z = 0;
    detected_armor_tf.transform.rotation.w = 0;
    tf_broadcaster->sendTransform(detected_armor_tf);

    // Publish TF filtered results
    geometry_msgs::msg::TransformStamped filtered_armor_tf;
    filtered_armor_tf.header.stamp = header.stamp;
    filtered_armor_tf.header.frame_id = "camera_link";
    filtered_armor_tf.child_frame_id = "filtered_armor";
    filtered_armor_tf.transform.translation.x = dst_[0] / 1000;
    filtered_armor_tf.transform.translation.y = dst_[1] / 1000;
    filtered_armor_tf.transform.translation.z = dst_[2] / 1000;
    filtered_armor_tf.transform.rotation.x = 1;
    filtered_armor_tf.transform.rotation.y = 0;
    filtered_armor_tf.transform.rotation.z = 0;
    filtered_armor_tf.transform.rotation.w = 0;
    tf_broadcaster->sendTransform(filtered_armor_tf);

    // Publish TF predictedion after 0.3s
    geometry_msgs::msg::TransformStamped predicted_armor_tf;
    predicted_armor_tf.header.stamp = header.stamp;
    predicted_armor_tf.header.frame_id = "camera_link";
    predicted_armor_tf.child_frame_id = "predicted_armor";
    predicted_armor_tf.transform.translation.x = (dst_[0] + dst_[3] * 0.5) / 1000;
    predicted_armor_tf.transform.translation.y = (dst_[1] + dst_[4] * 0.5) / 1000;
    predicted_armor_tf.transform.translation.z = (dst_[2] + dst_[5] * 0.5) / 1000;
    predicted_armor_tf.transform.rotation.x = 1;
    predicted_armor_tf.transform.rotation.y = 0;
    predicted_armor_tf.transform.rotation.z = 0;
    predicted_armor_tf.transform.rotation.w = 0;
    tf_broadcaster->sendTransform(predicted_armor_tf);
}

/**
 * @brief Callback function when receiving armor key points from armor detector node
 *
 * The callback performs PNP solving, kalman filtering, and determine when to fire.
 *
 * @param key_points_msg Key point messagge
 */
void PNPSolverNode::keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr key_points_msg)
{
    std::vector<float> points;
    int points_size = key_points_msg->points.size();
    bool large_armor = key_points_msg->large_armor;

    for (int i = 0; i < 8; i++)
    {
        points.push_back(key_points_msg->points[i]);
    }

    Coordinates coordinates = pnp_solver_.getArmorCoordinates(points, points_size, large_armor);

    if (coordinates.tvec.size() == 0)
    {
        publishZeroPredictedArmor(key_points_msg->header);
        return;
    }

    vision_msgs::msg::PredictedArmor predicted_armor_msg;
    this->validateCoordinates(&predicted_armor_msg, key_points_msg->header, coordinates.tvec);
    this->publishPredictedArmor(&predicted_armor_msg, key_points_msg->header, coordinates.tvec, coordinates.rvec);

    if (publish_tf_)
    {
        this->publishTF(key_points_msg->header, coordinates.tvec);
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "First TF published");

    if (seq_ % 1000 == 0)
    {
        RCLCPP_INFO(this->get_logger(), "seq: %d", seq_);
    }
    seq_++;
}

/**
 * @brief Publishes a message to stop firing if one second have passed the last fire time
 * 
 */
void PNPSolverNode::check_last_firing_time()
{
    if (this->now().seconds() - last_fire_time > 1 && locked_in_frames > num_frames_to_fire_after)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping firing");
        std_msgs::msg::String auto_aim_tracking_status_msg;
        auto_aim_tracking_status_msg.data = "STOPPING_FIRING";
        auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        locked_in_frames = 0;
        publishZeroPredictedArmor(std_msgs::msg::Header());
    }
}

/**
 * @brief Publishes a predicted armor message with all transformation initialized to 0
 * 
 * @param header The header to be published
 */
void PNPSolverNode::publishZeroPredictedArmor(std_msgs::msg::Header header)
{
    vision_msgs::msg::PredictedArmor predicted_armor_msg;
    predicted_armor_msg.header = header;
    predicted_armor_msg.x = 0;
    predicted_armor_msg.y = 0;
    predicted_armor_msg.z = 0;
    predicted_armor_msg.rvec_x = 0;
    predicted_armor_msg.rvec_y = 0;
    predicted_armor_msg.rvec_z = 0;
    predicted_armor_msg.x_vel = 0;
    predicted_armor_msg.y_vel = 0;
    predicted_armor_msg.z_vel = 0;
    predicted_armor_msg.fire = locked_in_frames > num_frames_to_fire_after; // allows firing to still occur until the target is lost for 2 seconds and locked_in_frames is reset

    predicted_armor_publisher->publish(predicted_armor_msg);
}

int main(int argc, char *argv[])
{

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto pnp_solver_node = std::make_shared<PNPSolverNode>(options);

    exec.add_node(pnp_solver_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
