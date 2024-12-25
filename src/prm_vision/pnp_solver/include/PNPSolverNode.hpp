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

#include "PNPSolver.hpp"
#include "KalmanFilter.hpp"
#include "ValidityFilter.hpp"

#define PI 3.141592653589793238462643383

#define X_OFFSET 0 // Right of sensor
#define Y_OFFSET 0 // Down of sensor
#define Z_OFFSET 0 // Out of sensor

using namespace std::chrono_literals;

class PNPSolverNode : public rclcpp::Node
{
public:
    PNPSolverNode(const rclcpp::NodeOptions &);
    ~PNPSolverNode();

private:
    rclcpp::Subscription<vision_msgs::msg::KeyPoints>::SharedPtr key_points_subscriber;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::PredictedArmor>> predicted_armor_publisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> auto_aim_tracking_status_publisher = NULL;

    uint32_t seq_;

    PNPSolver pnp_solver_;
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
    void validateCoordinates(vision_msgs::msg::PredictedArmor* predicted_armor_msg, std_msgs::msg::Header header, std::vector<float> tvec);
    void publishPredictedArmor(vision_msgs::msg::PredictedArmor* predicted_armor_msg, std_msgs::msg::Header header, std::vector<float> tvec, std::vector<float> rvec);
    void publishTF(std_msgs::msg::Header header, std::vector<float> tvec);
    void publishZeroPredictedArmor(std_msgs::msg::Header header);
    void check_last_firing_time();
};
