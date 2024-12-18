#ifndef _TIME_DEBUG_H
#define _TIME_DEBUG_H

#define START_TIME() auto start = std::chrono::steady_clock::now();
#define END_TIME() auto end = std::chrono::steady_clock::now();
#define ROS_LOG_DURATION() RCLCPP_INFO(this->get_logger(), "duration: %dns", std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
#define PRINT_DURATION() std::cout << "duration: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << "ns" << std::endl;

#endif // _TIME_DEBUG_H