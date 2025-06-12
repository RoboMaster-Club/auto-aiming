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
	bool started = control_communicator->start_uart_connection(this->port);
	RCLCPP_INFO(this->get_logger(), "UART Connection %s", started ? "Started" : "Failed");

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
	this->health_publisher = this->create_publisher <std_msgs::msg::Int16>("health", 1);
	this->uart_read_timer = this->create_wall_timer(4ms, std::bind(&ControlCommunicatorNode::read_uart, this));

	RCLCPP_INFO(this->get_logger(), "Control Communicator Node Started.");
}

ControlCommunicatorNode::~ControlCommunicatorNode()
{
	close(control_communicator->port_fd);
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
	if (!control_communicator->is_connected || control_communicator->port_fd < 0)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, ignoring aim message.");
		return;
	}

	float yaw, pitch;
	control_communicator->compute_aim(aim_bullet_speed, msg->x, msg->y, msg->z, yaw, pitch);

	PackageOut package;
	package.frame_id = 0xAA;
	package.frame_type = FRAME_TYPE_AUTO_AIM;
	package.autoAimPackage.yaw = yaw;
	package.autoAimPackage.pitch = pitch;
	package.autoAimPackage.fire = 1;

	int bytes_written = write(control_communicator->port_fd, &package, sizeof(PackageOut));
	fsync(control_communicator->port_fd);

	float dst = sqrt(pow(msg->x, 2) + pow(msg->y, 2) + pow(msg->z, 2));
	if (this->auto_aim_frame_id % 100 == 0 && this->auto_aim_frame_id != 0 && dst > 0)
	{
		RCLCPP_INFO(this->get_logger(), "Yaw: %.2f | Pitch: %.2f | dst: %.2f | (x, y, z): (%.2f, %.2f, %.2f)", yaw, pitch, dst, msg->x, msg->y, msg->z);
	}
	
	if (bytes_written != sizeof(PackageOut))
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to write complete package to UART. Bytes written: %d, Expected: %lu", bytes_written, sizeof(PackageOut));
		return;
	}

	auto_aim_frame_id++;
}

void ControlCommunicatorNode::nav_handler(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
	if (!control_communicator->is_connected || control_communicator->port_fd < 0)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, ignoring nav message %d.", this->nav_frame_id++);
		return;
	}

	PackageOut package;

	package.frame_id = 0xAA;
	package.frame_type = FRAME_TYPE_NAV;
	package.navPackage.x_vel = msg->linear.x;
	package.navPackage.y_vel = msg->linear.y;
	package.navPackage.yaw_rad = msg->angular.z;
	package.navPackage.state = 2;
	write(control_communicator->port_fd, &package, sizeof(PackageOut));
	fsync(control_communicator->port_fd);

	RCLCPP_INFO(this->get_logger(), "x_vel = %f, y_vel = %f, yaw = %f", package.navPackage.x_vel, package.navPackage.y_vel, package.navPackage.yaw_rad);
}

void ControlCommunicatorNode::heart_beat_handler()
{
	if (!control_communicator->is_connected || control_communicator->port_fd < -1)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, trying to reconnect.");
		control_communicator->start_uart_connection(this->port);
	}

	PackageOut package;
	this->heart_beat_frame_id++;
	package.frame_id = 0xAA;
	package.frame_type = FRAME_TYPE_HEART_BEAT;
	package.heartBeatPackage._a = 0xAA;
	package.heartBeatPackage._b = 0xAA;
	package.heartBeatPackage._c = 0xAA;
	package.heartBeatPackage._d = 0xAA;
	int success = write(control_communicator->port_fd, &package, sizeof(PackageOut));
	fsync(control_communicator->port_fd);
	if (success == -1)
	{
		control_communicator->is_connected = false;
		RCLCPP_ERROR(this->get_logger(), "Error %i from write: %s", errno, strerror(errno));
		control_communicator->start_uart_connection(this->port);
	}
	if (this->heart_beat_frame_id % 10 == 0)
	{
		RCLCPP_INFO(this->get_logger(), "Heart Beat %d", this->heart_beat_frame_id);
	}
}

void ControlCommunicatorNode::read_uart()
{
	PackageIn package;
	int success = read(control_communicator->port_fd, &package, sizeof(PackageIn));

	rclcpp::Time curr_time = this->now();
	if (success == -1)
	{
		RCLCPP_ERROR(this->get_logger(), "Error %i from read: %s", errno, strerror(errno));
		return;
	}

	if (package.head != 0xAA) // Package validation
	{
		RCLCPP_WARN(this->get_logger(), "Packet miss aligned.");
		if (this->read_alignment())
		{
			RCLCPP_INFO(this->get_logger(), "Read alignment success.");
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Read alignment failed.");
		}
		return;
	}

	// Handle TF
	this->pitch_vel = package.pitch_vel; 			// rad/s
	this->pitch = package.pitch;					// rad
	this->yaw_vel = package.yaw_vel;	   			// rad/s
	this->is_red = package.enemy_color;				// 1 for enemy is red
	this->is_match_running = package.match_start; 	// LSB denotes if match is started
	this->valid_read = true;


	// publishing color and match status and health
	std_msgs::msg::String target_robot_color;
    target_robot_color.data = this->is_red ? "red" : "blue";

	std_msgs::msg::Bool match_status;
	match_status.data = this->is_match_running;
	std_msgs::msg::Int16 health;
	health.data = package.hp;

	// publishers
	health_publisher->publish(health);
	match_status_publisher->publish(match_status);

	if (this->old_target_robot_color != target_robot_color.data)
	{
		this->old_target_robot_color = target_robot_color.data;
		RCLCPP_INFO(this->get_logger(), "Target Robot Color: %s", target_robot_color.data.c_str());
	}

	geometry_msgs::msg::TransformStamped pitch_tf;
	pitch_tf.header.stamp = curr_time;
	pitch_tf.header.frame_id = "yaw_link";
	pitch_tf.child_frame_id = "pitch_link";
	pitch_tf.transform.translation.x = 0;
	pitch_tf.transform.translation.y = 0;
	pitch_tf.transform.translation.z = 0;

	tf2::Quaternion pitch_q;
	pitch_q.setRPY(0, this->pitch, 0);
	pitch_tf.transform.rotation.x = pitch_q.x();
	pitch_tf.transform.rotation.y = pitch_q.y();
	pitch_tf.transform.rotation.z = pitch_q.z();
	pitch_tf.transform.rotation.w = pitch_q.w();

	tf_broadcaster->sendTransform(pitch_tf);

	// if abs of Odom is > 99999, must be a bad read
	if (abs(package.x) > 99999 || abs(package.y) > 99999 || abs(package.orientation) > 99999)
	{
		RCLCPP_WARN(this->get_logger(), "Bad Odom Read: (x, y, orientation): (%f, %f, %f)", package.x, package.y, package.orientation);
		return;
	}

	// Handle Odom
	geometry_msgs::msg::TransformStamped odom_tf;
	nav_msgs::msg::Odometry odom;

	odom_tf.header.stamp = curr_time;
	odom_tf.header.frame_id = "odom";
	odom_tf.child_frame_id = "base_footprint";
	odom_tf.transform.translation.x = package.x;
	odom_tf.transform.translation.y = package.y;
	odom_tf.transform.translation.z = 0;

	odom.header.stamp = curr_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	odom.pose.pose.position.x = package.x;
	odom.pose.pose.position.y = package.y;
	odom.pose.pose.position.z = 0;

	tf2::Quaternion odom_q;
	odom_q.setRPY(0, 0, package.orientation);

	odom_tf.transform.rotation.x = odom_q.x();
	odom_tf.transform.rotation.y = odom_q.y();
	odom_tf.transform.rotation.z = odom_q.z();
	odom_tf.transform.rotation.w = odom_q.w();

	odom.pose.pose.orientation.x = odom_q.x();
	odom.pose.pose.orientation.y = odom_q.y();
	odom.pose.pose.orientation.z = odom_q.z();
	odom.pose.pose.orientation.w = odom_q.w();

	odom.pose.covariance[0] = 0.01;
	odom.pose.covariance[7] = 0.01;
	odom.pose.covariance[14] = 1e13;
	odom.pose.covariance[21] = 1e13;
	odom.pose.covariance[28] = 1e13;
	odom.pose.covariance[35] = 0.01;

	odom.twist.twist.linear.x = package.x_vel;
	odom.twist.twist.linear.y = package.y_vel;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = this->yaw_vel;

	odom.twist.covariance[0] = 0.01;
	odom.twist.covariance[7] = 0.01;
	odom.twist.covariance[14] = 1e13;
	odom.twist.covariance[21] = 1e13;
	odom.twist.covariance[28] = 1e13;
	odom.twist.covariance[35] = 0.01;

	tf_broadcaster->sendTransform(odom_tf);

	this->odometry_publisher->publish(odom);

	return;
}

bool ControlCommunicatorNode::read_alignment()
{
	RCLCPP_INFO(this->get_logger(), "Attemp to alignment.");
	uint8_t i = 0;
	uint8_t buffer[35]; // we only read sizeof(packagein) - 1
	do
	{
		int success = read(control_communicator->port_fd, &(buffer[0]), sizeof(buffer[0]));
		if (success)
		{
			i++;
		}
	} while (buffer[0] != 0xAA || i > sizeof(PackageIn) * 2);
	read(control_communicator->port_fd, &buffer, sizeof(PackageIn) - 1);

	return i <= sizeof(PackageIn) * 2;
}


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ControlCommunicatorNode>("/dev/ttyTHS1");

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
