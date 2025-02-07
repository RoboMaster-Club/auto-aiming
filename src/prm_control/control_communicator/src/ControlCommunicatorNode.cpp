#include "ControlCommunicatorNode.hpp"

#include "utils.cpp"

#include <fcntl.h>	 // Contains file controls like O_RDWR
#include <errno.h>	 // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>	 // write(), read(), close()

#include "time_debug.h"

#define PI 3.141592653589793238462643383 // It was 4 AM when I wrote this.

using namespace std::chrono_literals;
using namespace std::placeholders;

ControlCommunicatorNode::ControlCommunicatorNode(const char *port) : Node("control_communicator_node")
{

	aim_stop_null_frame_count = this->declare_parameter("aim.stop_null_frame_count", 3);

	aim_bullet_speed = this->declare_parameter("aim.bullet_speed", 16.0f);

	this->port = port;

	this->start_uart(port);

	if (this->read_alignment())
	{
		RCLCPP_INFO(this->get_logger(), "Inital Read alignment success.");
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "Inital Read alignment failed.");
	}

	RCLCPP_INFO(this->get_logger(), "should have printed.");

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


	this->nav_subscriber = this->create_subscription<geometry_msgs::msg::Twist>( // should not be twiststamped
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

void ControlCommunicatorNode::start_uart(const char *port)
{
	this->is_connected = false;
	this->port_fd = open(port, O_RDWR);

	// Check for errors
	if (this->port_fd < 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to open: %s, %s", port, strerror(errno));
		return;
	}

	struct termios tty;

	// Set UART TTY to 8n1
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	tty.c_cflag &= ~CRTSCTS;	   // No RTS/CTS flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
	tty.c_lflag &= ~ICANON;		   // Disable canonical mode

	// Disable echo, erasure and newline echo
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;

	// Disable interpretation of INTR, QUIT and SUSP
	tty.c_lflag &= ~ISIG;

	// Disable special handling, interpretation, S/W flow control, \n conv.
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;

	tty.c_cc[VTIME] = 10;				// Wait for up to 1s (10 deciseconds)
	tty.c_cc[VMIN] = sizeof(PackageIn); // Block for sizeof(PackageOut) bits

	// Set the baud rate
	cfsetispeed(&tty, B1152000);

	// Save tty settings, also checking for error
	if (tcsetattr(this->port_fd, TCSANOW, &tty) != 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
		return;
	}
	this->is_connected = true;
	RCLCPP_INFO(this->get_logger(), "UART Connected");

	return;
}

void ControlCommunicatorNode::auto_aim_handler(const std::shared_ptr<vision_msgs::msg::PredictedArmor> msg)
{
	if (!is_connected || this->port_fd < 0)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, ignoring aim message %d.", this->auto_aim_frame_id++);
		return;
	}
	rclcpp::Time curr_time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
	float dt = (curr_time.seconds() - this->now().seconds()) * 1000; // in ms

	// yaw/pitch and XYZ
	vec3 P(msg->x/1000, msg->y/1000, msg->z/1000), V(0,0, 0), grav(0, 0, 9.81);
    	double p = 0, y = 0; bool im = 0;
	
	float yaw;
	float pitch;
	float dst;

	if (msg->x != 0 && msg->z != 0)
	{
		dst = sqrt(pow(msg->x, 2) + pow(msg->y, 2) + pow(msg->z, 2));
		dt = 0;
		float pred_x = msg->x;
		float pred_y = msg->y;
		float pred_z = msg->z;
		yaw = -atan(pred_y / pred_x) * 180 / PI;
		pitch = atan(pred_z / pred_x) * 180 / PI;

    		pitch_yaw_gravity_model_movingtarget_const_v(P, V, grav, 0, &p, &y, &im); y = y * (msg->y > 0 ? -1 : 1);  //currently a bug where yaw is never negative, so we just multiply by the sign of "y" of the target
	}
	else
	{
		yaw = 0;
		pitch = 0;
		dst = 0;
	}

	PackageOut package;
	this->auto_aim_frame_id++;
	package.frame_id = 0xAA;
	package.frame_type = FRAME_TYPE_AUTO_AIM;
	package.autoAimPackage.yaw = y;
	package.autoAimPackage.pitch = p;
	// for safety this is commented out unless on sentry    package.autoAimPackage.fire = msg->fire;
	write(this->port_fd, &package, sizeof(PackageOut));
	fsync(this->port_fd);
	RCLCPP_INFO(this->get_logger(), "Yaw, Pitch: %.3f, %.3f, FIRE=%.3f", yaw, pitch, msg->fire);
	RCLCPP_INFO_ONCE(this->get_logger(), "First auto aim pkg sent.");
}

void ControlCommunicatorNode::nav_handler(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
	if (!is_connected || this->port_fd < 0)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, ignoring nav message %d.", this->nav_frame_id++);
		return;
	}

	PackageOut package;

	package.frame_id = 0xAA; //(uint8_t)this->nav_frame_id++; ANOTHER CHANGE
	package.frame_type = FRAME_TYPE_NAV;
	package.navPackage.x_vel = msg->linear.x;
	package.navPackage.y_vel = msg->linear.y;
	package.navPackage.yaw_rad = msg->angular.z;
	package.navPackage.state = 1;
	write(this->port_fd, &package, sizeof(PackageOut));
	fsync(this->port_fd);

	RCLCPP_INFO(this->get_logger(), "x_vel = %f, y_vel = %f, yaw = %f", package.navPackage.x_vel, package.navPackage.y_vel, package.navPackage.yaw_rad);
}


void ControlCommunicatorNode::heart_beat_handler()
{
	if (!this->is_connected || this->port_fd < -1)
	{
		RCLCPP_WARN(this->get_logger(), "UART Not connected, trying to reconnect.");
		this->start_uart(this->port);
	}

	// CHANGED FRAME ID TO 0xAA TO COMPLY WITH LEO

	PackageOut package;
	this->heart_beat_frame_id++;
	package.frame_id = 0xAA;
	package.frame_type = FRAME_TYPE_HEART_BEAT;
	package.heartBeatPackage._a = 0xAA;
	package.heartBeatPackage._b = 0xAA;
	package.heartBeatPackage._c = 0xAA;
	package.heartBeatPackage._d = 0xAA;
	int success = write(this->port_fd, &package, sizeof(PackageOut));
	fsync(this->port_fd);
	if (success == -1)
	{
		this->is_connected = false;
		RCLCPP_ERROR(this->get_logger(), "Erro	r %i from write: %s", errno, strerror(errno));
		start_uart(this->port);
	}
	if (this->heart_beat_frame_id % 10 == 0)
	{
		RCLCPP_INFO(this->get_logger(), "Heart Beat %d", this->heart_beat_frame_id);
	}
}

bool ControlCommunicatorNode::read_alignment()
{
	RCLCPP_INFO(this->get_logger(), "Attemp to alignment.");
	uint8_t i = 0;
	uint8_t buffer[33];
	do
	{
		int success = read(this->port_fd, &(buffer[0]), sizeof(buffer[0]));
		if (success)
		{
			i++;
		}
	} while (buffer[0] != 0xAA || i > sizeof(PackageIn) * 2);
	read(this->port_fd, &buffer, sizeof(PackageIn) - 1);

	return i <= sizeof(PackageIn) * 2;
}

void ControlCommunicatorNode::read_uart()
{
	PackageIn package;
	int success = read(this->port_fd, &package, sizeof(PackageIn));

	rclcpp::Time curr_time = this->now();
	if (success == -1)
	{
		RCLCPP_ERROR(this->get_logger(), "Error %i from read: %s", errno, strerror(errno));
		return;
	}

	// // print each field of package
	// RCLCPP_INFO(this->get_logger(), "\n");
	// RCLCPP_INFO(this->get_logger(), "head: %x", package.head);
	// RCLCPP_INFO(this->get_logger(), "pitch: %f", package.pitch);
	// RCLCPP_INFO(this->get_logger(), "pitch_vel: %f", package.pitch_vel);
	// RCLCPP_INFO(this->get_logger(), "yaw_vel: %f", package.yaw_vel);
	// RCLCPP_INFO(this->get_logger(), "x: %f", package.x);
	// RCLCPP_INFO(this->get_logger(), "y: %f", package.y);
	// RCLCPP_INFO(this->get_logger(), "orientation: %f", package.orientation);
	// RCLCPP_INFO(this->get_logger(), "x_vel: %f", package.x_vel);
	// RCLCPP_INFO(this->get_logger(), "y_vel: %f", package.y_vel);
	// RCLCPP_INFO(this->get_logger(), "\n");

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
	this->pitch_vel = package.pitch_vel; // rad/s
	this->pitch = package.pitch;			   // rad
	this->yaw_vel = package.yaw_vel;	   // rad/s
	this->is_red = package.ref_flags & 2;	// second lowest  denotes if enemy is red
	this->is_match_running = package.ref_flags & 1;	// LSB denotes if match is started
	this->valid_read = true;

	// publishing color and match status
	std_msgs::msg::String target_robot_color;
        target_robot_color.data = this->is_red ? "red" : "blue";
        //target_robot_color_publisher->publish(target_robot_color);
	std_msgs::msg::Bool match_status;
	match_status.data = this->is_match_running;
	//match_status_publisher->publish(match_status);

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

	recive_frame_id++;

	return;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ControlCommunicatorNode>("/dev/ttyTHS0");

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
