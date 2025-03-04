#include "ControlCommunicatorNode.hpp"
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#define PI 3.141592653589793238462643383 // It was 4 AM when I wrote this.
using namespace std::chrono_literals;
using namespace std::placeholders;

ControlCommunicatorNode::ControlCommunicatorNode(const char *port) : Node("control_communicator_node")
{
	aim_stop_null_frame_count = this->declare_parameter("aim.stop_null_frame_count", 3);
	aim_bullet_speed = this->declare_parameter("aim.bullet_speed", 16.0f);
	this->port = port;
	this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	this->tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

	this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

	publish_static_tf(158.7f / 1000.f, 0.f / 1000.f, 47.5 / 1000.f, 0, 0, 0, "pitch_link", "camera_link");
	publish_static_tf(0, 0, 478.f / 1000.f, 0, 0, 0, "base_link", "yaw_link");
	publish_static_tf(0, 0, 0, 0, 0, 0, "base_footprint", "base_link");
	publish_static_tf(0, 0, 0.3, 0, 0, 0, "base_link", "laser");

	this->heart_beat_timer = this->create_wall_timer(1000ms, std::bind(&ControlCommunicatorNode::heart_beat_handler, this));
	this->auto_aim_subscriber = this->create_subscription<vision_msgs::msg::PredictedArmor>(
		"predicted_armor", 1, std::bind(&ControlCommunicatorNode::auto_aim_handler, this, _1));
	this->nav_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel", 1, std::bind(&ControlCommunicatorNode::nav_handler, this, _1));
	this->odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
	this->target_robot_color_publisher = this->create_publisher<std_msgs::msg::String>("color_set", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
	this->match_status_publisher = this->create_publisher<std_msgs::msg::Bool>("match_start", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
	this->uart_read_timer = this->create_wall_timer(4ms, std::bind(&ControlCommunicatorNode::read_uart, this));

	RCLCPP_INFO(this->get_logger(), "Control Communicator Node Started.");
}

ControlCommunicatorNode::~ControlCommunicatorNode()
{
	close(this->port_fd);
}

void ControlCommunicatorNode::publish_static_tf(float x, float y, float z, float roll, float pitch, float yaw, const char *frame_id, const char *child_frame_id)
{
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = frame_id;
	t.child_frame_id = child_frame_id;
	t.transform.translation.x = x;
	t.transform.translation.y = y;
	t.transform.translation.z = z;

	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();

	tf_static_broadcaster->sendTransform(t);
}

void ControlCommunicatorNode::auto_aim_handler(const std::shared_ptr<vision_msgs::msg::PredictedArmor> msg)
{
	if (!is_connected || this->port_fd < 0)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, ignoring aim message.");
		return;
	}

	float yaw, pitch;
	compute_aim(aim_bullet_speed, msg->x, msg->y, msg->z, yaw, pitch);

	PackageOut package;
	package.frame_id = 0xAA;
	package.frame_type = FRAME_TYPE_AUTO_AIM;
	package.autoAimPackage.yaw = yaw;
	package.autoAimPackage.pitch = pitch;
	package.autoAimPackage.fire = 1;
	write(this->port_fd, &package, sizeof(PackageOut));
	fsync(this->port_fd);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ControlCommunicatorNode>("/dev/ttyTHS1");

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
