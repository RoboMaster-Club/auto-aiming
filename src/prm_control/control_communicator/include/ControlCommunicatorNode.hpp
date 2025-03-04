#ifndef CONTROL_COMMUNICATOR_NODE_HPP
#define CONTROL_COMMUNICATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/predicted_armor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <memory>
#include <functional>
#include <string>
#include <chrono>

#include "ControlCommunicator.h"

class ControlCommunicatorNode : public rclcpp::Node
{
public:
	ControlCommunicatorNode(const char *port);
	~ControlCommunicatorNode();

private:
	void publish_static_tf(float x, float y, float z, float roll, float pitch, float yaw, const char *frame_id, const char *child_frame_id);
	void auto_aim_handler(const std::shared_ptr<vision_msgs::msg::PredictedArmor> msg);
	void nav_handler(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
	void heart_beat_handler();
	void read_uart();

	int port_fd;
	bool is_connected;
	float aim_bullet_speed;
	int aim_stop_null_frame_count;
	std::string port;
	rclcpp::TimerBase::SharedPtr heart_beat_timer;
	rclcpp::Subscription<vision_msgs::msg::PredictedArmor>::SharedPtr auto_aim_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_robot_color_publisher;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr match_status_publisher;
	rclcpp::TimerBase::SharedPtr uart_read_timer;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};
#endif // CONTROL_COMMUNICATOR_NODE_HPP