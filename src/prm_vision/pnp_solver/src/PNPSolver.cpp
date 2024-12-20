#include "PNPSolver.hpp"

PNPSolver::PNPSolver(const rclcpp::NodeOptions &options) : Node("pnp_solver", options)
{

    publish_tf_ = this->declare_parameter("publish_tf", false);
    publish_tf_ = true;
    RCLCPP_INFO(this->get_logger(), "Publish TF: %s", publish_tf_ ? "true" : "false");

    // +X: Out, +Y: Right, +Z: Up
    small_armor_object_points.emplace_back(cv::Point3f(0, -SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));  // Top Left
    small_armor_object_points.emplace_back(cv::Point3f(0, -SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT)); // Bot Left
    small_armor_object_points.emplace_back(cv::Point3f(0, SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));   // Top Right
    small_armor_object_points.emplace_back(cv::Point3f(0, SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT));  // Bot Right

    large_armor_object_points.emplace_back(cv::Point3f(0, -LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));  // Top Left
    large_armor_object_points.emplace_back(cv::Point3f(0, -LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT));  // Bot Left
    large_armor_object_points.emplace_back(cv::Point3f(0, LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));  // Top Righy
    large_armor_object_points.emplace_back(cv::Point3f(0, LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT));  // Bot Right

    camera_matrix_[0][0] = 1019.108731;
    camera_matrix_[0][1] = 0;
    camera_matrix_[0][2] = 601.884969;
    camera_matrix_[1][0] = 0;
    camera_matrix_[1][1] = 1016.784980;
    camera_matrix_[1][2] = 521.004587;
    camera_matrix_[2][0] = 0;
    camera_matrix_[2][1] = 0;
    camera_matrix_[2][2] = 1;


    kalman_filter_ = KalmanFilter();
    validity_filter_ = ValidityFilter();

    camera_matrix = cv::Mat(3, 3, CV_32F, &camera_matrix_);

    distortion_coefficient = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
    distortion_coefficient.at<double>(0, 0) = -0.108767;
    distortion_coefficient.at<double>(1, 0) = -0.072085;
    distortion_coefficient.at<double>(2, 0) = -0.000847;
    distortion_coefficient.at<double>(3, 0) = 0.0;

    rvec = cv::Mat(3, 1, CV_32F);
    tvec = cv::Mat(3, 1, CV_32F);

    seq_ = 0;

    prev_time_ = this->now();

    if (publish_tf_)
    {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // STOP FIRING if no target detected for 2 seconds
    this->firing_timer = this->create_wall_timer(500ms, std::bind(&PNPSolver::check_last_firing_time, this));

    // Initialize subscriber and publisher for predicted armor, key points, and auto aim tracking status
    predicted_armor_publisher = this->create_publisher<vision_msgs::msg::PredictedArmor>("predicted_armor", 10);
    auto_aim_tracking_status_publisher = this->create_publisher<std_msgs::msg::String>("auto_aim_tracking_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    key_points_subscriber = this->create_subscription<vision_msgs::msg::KeyPoints>(
        "key_points", 10, std::bind(&PNPSolver::keyPointsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PNP Solver Created");
}

PNPSolver::~PNPSolver()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Destorying PNP Solver");
    return;
}

/**
 * @brief Callback function when receiving armor key points from armor detector node
 * 
 * The callback performs PNP solving, kalman filtering, and determine when to fire.
 * 
 * @param key_points_msg key point messagge
 */
void PNPSolver::keyPointsCallback(const vision_msgs::msg::KeyPoints::SharedPtr key_points_msg)
{

    if (key_points_msg->points.size() != 8 || key_points_msg->points[0] == 0)
    {
        publishZeroPredictedArmor(key_points_msg->header);
        return;
    }

    std::vector<cv::Point2f> image_points;
    image_points.push_back(cv::Point2f(key_points_msg->points[0], key_points_msg->points[1]));
    image_points.push_back(cv::Point2f(key_points_msg->points[2], key_points_msg->points[3]));
    image_points.push_back(cv::Point2f(key_points_msg->points[4], key_points_msg->points[5]));
    image_points.push_back(cv::Point2f(key_points_msg->points[6], key_points_msg->points[7]));

    bool result = cv::solvePnP(small_armor_object_points, image_points, camera_matrix, distortion_coefficient, rvec, tvec, false, cv::SOLVEPNP_IPPE); 

    if (!result)
    {
        RCLCPP_WARN(this->get_logger(), "Solve Failed on %d", seq_++);
        publishZeroPredictedArmor(key_points_msg->header);
        return;
    }

    rclcpp::Time curr_time_(key_points_msg->header.stamp.sec, key_points_msg->header.stamp.nanosec, RCL_ROS_TIME);

    double dt = (curr_time_.seconds() - prev_time_.seconds()); // in s

    float Y = -tvec.at<float>(0, 0);
    float Z = -tvec.at<float>(1, 0);
    float X = tvec.at<float>(2, 0);
    tvec.at<float>(0, 0) = X;
    tvec.at<float>(1, 0) = Y;
    tvec.at<float>(2, 0) = Z;

    std_msgs::msg::String auto_aim_tracking_status_msg;

    bool reset_kalman = validity_filter_.validation(X, Y, Z, dt);
    if (reset_kalman)
    {
        auto_aim_tracking_status_msg.data = "RESET KALMAN";
        auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        kalman_filter_.reset();
        RCLCPP_INFO(this->get_logger(), "Validity Filter Reset Kalman Filter");
        prev_time_ = curr_time_;
    }

    // Stopping motor
    if (validity_filter_.state == STOPPING)
    {
        auto_aim_tracking_status_msg.data = "STOPPING";
        auto_aim_tracking_status_publisher->publish(auto_aim_tracking_status_msg);
        publishZeroPredictedArmor(key_points_msg->header);
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

        kalman_filter_.update(X, Y, Z, dt, dst_);
        prev_time_ = curr_time_;
        locked_in_frames++;
    }


    // Publish Predicted Armor if valid and publish
    vision_msgs::msg::PredictedArmor predicted_armor_msg;
    predicted_armor_msg.header = key_points_msg->header;

    // Translation XYZ
    // disable kalman for now
    double distance = sqrt(X*X + Y*Y + Z*Z);
    predicted_armor_msg.x = X;
    predicted_armor_msg.y = Y + 50;
    predicted_armor_msg.z = Z - 60; // 50mm coordinate transform in Z     // + (0.1*distance + -127.75);

    vec3 P(X/1000, Y/1000, Z/1000), V(0,0, 0), grav(0, 0, 9.81);
    double p = 0, y = 0; bool im = 0;
    pitch_yaw_gravity_model_movingtarget_const_v(P, V, grav, 0, &p, &y, &im); y = y * (Y > 0 ? -1 : 1);  //currently a bug where yaw is never negative, so we just multiply by the sign of "y" of the target

    double yaw = -atan(Y / X) * 180 / PI;
    double pitch = atan(Z / X) * 180 / PI;

    // fire at the target
    predicted_armor_msg.fire = locked_in_frames > num_frames_to_fire_after;

    // Rotation XYZ
    // NOTE: Eventually this should be passed into the kalman filter and predicted_rvec should be published instead
    predicted_armor_msg.rvec_x = rvec.at<float>(0, 0);
    predicted_armor_msg.rvec_y = rvec.at<float>(1, 0);
    predicted_armor_msg.rvec_z = rvec.at<float>(2, 0);

    if (reset_kalman)
    {
        predicted_armor_msg.x_vel = 0;
        predicted_armor_msg.y_vel = 0;
        predicted_armor_msg.z_vel = 0;
    }
    else
    {
        predicted_armor_msg.x_vel = dst_[0];
        predicted_armor_msg.y_vel = dst_[1];
        predicted_armor_msg.z_vel = dst_[2];
    }
    predicted_armor_publisher->publish(predicted_armor_msg);

    // Publish TF raw results
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped detected_armor_tf;

        detected_armor_tf.header.stamp = key_points_msg->header.stamp;
        detected_armor_tf.header.frame_id = "camera_link";
        detected_armor_tf.child_frame_id = "detected_armor";
        detected_armor_tf.transform.translation.x = X / 1000;
        detected_armor_tf.transform.translation.y = Y / 1000;
        detected_armor_tf.transform.translation.z = Z / 1000;

        detected_armor_tf.transform.rotation.x = 1;
        detected_armor_tf.transform.rotation.y = 0;
        detected_armor_tf.transform.rotation.z = 0;
        detected_armor_tf.transform.rotation.w = 0;

        tf_broadcaster->sendTransform(detected_armor_tf);
    }

    // Publish TF filtered results
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped filtered_armor_tf;

        filtered_armor_tf.header.stamp = key_points_msg->header.stamp;
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
    }

    // Publish TF predictedion after 0.3s
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped predicted_armor_tf;

        predicted_armor_tf.header.stamp = key_points_msg->header.stamp;
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

    RCLCPP_INFO_ONCE(this->get_logger(), "First TF published");

    if (seq_ % 1000 == 0)
    {
        RCLCPP_INFO(this->get_logger(), "seq: %d", seq_);
    }
    seq_++;
}

void PNPSolver::check_last_firing_time()
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

void PNPSolver::publishZeroPredictedArmor(std_msgs::msg::Header header)
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
    auto pnp_solver_node = std::make_shared<PNPSolver>(options);

    exec.add_node(pnp_solver_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
