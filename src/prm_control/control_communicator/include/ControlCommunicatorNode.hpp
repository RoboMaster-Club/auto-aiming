#ifndef _CONTROL_COMMUNICATOR_NODE_H
#define _CONTROL_COMMUNICATOR_NODE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <math.h>

#include <errno.h> // Error integer and strerror() function

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "projectile_angle_convel.h"

#include "vision_msgs/msg/yaw_pitch.hpp"
#include "vision_msgs/msg/predicted_armor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "messages.h"
#include "PitchLookupModel.hpp"

// using namespace std::chrono_literals;
// using namespace std::placeholders;
// namespace rmoss_msgs = rmoss_interfaces::msg;

class ControlCommunicatorNode : public rclcpp::Node
{
public:
	ControlCommunicatorNode(const char *port);

	~ControlCommunicatorNode();

private:
	float yaw_vel = 0;	 // rad/s (+ccw, -cw)
	float pitch_vel = 0; // rad/s
	float pitch = 0;	 // rad (+up, -down)?
	bool is_red = 0;
	bool is_match_running = 0;
	bool valid_read = false;

	uint32_t recive_frame_id = 0;

	const char *port;
	int port_fd = -1;
	bool is_connected = false;

	uint32_t auto_aim_frame_id = 0;
	uint32_t nav_frame_id = 0;
	uint32_t heart_beat_frame_id = 0;

	int aim_stop_null_frame_count;
	int aim_null_frame_count = 0;

	float aim_bullet_speed; // mm/s

	int8_t curr_pois = 0;
	bool right = true;

	std::string lookup_table_path;
    PitchLookupModel pitch_lookup_model;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;

	rclcpp::Subscription<vision_msgs::msg::PredictedArmor>::SharedPtr auto_aim_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
	std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> target_robot_color_publisher = NULL;
	std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> match_status_publisher = NULL;


	rclcpp::TimerBase::SharedPtr uart_read_timer;
	rclcpp::TimerBase::SharedPtr read_lookup_table_time;
	rclcpp::TimerBase::SharedPtr heart_beat_timer;

	bool start_uart(const char *port);

	void publish_static_tf(float, float, float, float, float, float, const char *, const char *);

	void auto_aim_handler(const std::shared_ptr<vision_msgs::msg::PredictedArmor> msg);
	void nav_handler(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
	void heart_beat_handler();

	void read_uart();
};

#endif // CONTROL_COMMUNICATOR_NODE_H
