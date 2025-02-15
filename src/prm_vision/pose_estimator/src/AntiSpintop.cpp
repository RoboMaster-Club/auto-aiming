#include "AntiSpintop.hpp"

AntiSpintop::AntiSpintop() {
    AntiSpintop(10, 0.1);
}
/**
 * Constructs the anti spintop object
 * 
 * @param buffer_size buffer size for spin top detection
 * @param slope_threshold the yaw change threshold for identifying spinning behavior
*/
AntiSpintop::AntiSpintop(size_t buffer_size, double slope_threshold) {
    this->spin_direction = 0;
    this->buffer_size = buffer_size;
    this->slope_threshold = slope_threshold;
    this->target_radius = INITIAL_RADIUS_ESTIMATE;
    this->last_update_time = std::chrono::steady_clock::now();
}

/**
 * Checks if the robot is spinning
 * 
 * @param yaw yaw of the armor plate
 * @param threshold the threshold for yaw change
 * @returns 1 if robot is spinning counterclockwise, -1 if the robot is spinning clockwise
*/
int AntiSpintop::check_spinning(double yaw, double threshold) {
    this->current_yaw = yaw;
    this->yaw_buffer.push_back(yaw);

    // maintain the buffer size to be under limit
    if (this->yaw_buffer.size() > this->buffer_size) {
        this->yaw_buffer.erase(this->yaw_buffer.begin());
    }

    // calculate boxcar average for yaw
    double slope_sum = 0.0;
    int direction_sum = 0;
    int count = 0;
    for (size_t i = 1; i < this->yaw_buffer.size(); ++i) {
        double slope = abs(this->yaw_buffer[i] - this->yaw_buffer[i - 1]) / this->delta_time;

        // ignoring slope outliers
        if (slope < std::max(2 * this->boxcar_average, this->boxcar_average + 0.2)) {
            slope_sum += slope;
            count += 1;

            if (this->yaw_buffer[i] - this->yaw_buffer[i - 1] > 0) {
                direction_sum += 1;
            } else {
                direction_sum -= 1;
            }
        }
    }
    if (count > 0) {
        this->boxcar_average = slope_sum / count;
    } else {
        this->boxcar_average = 0;
    }

    // determine spin direction
    if (boxcar_average > threshold) {
        if (direction_sum < 0) {
            return -1;
        } else if (direction_sum > 0) {
            return 1;
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}

/**
 * Approximates the time for the armor to reach the optimal aiming position
 * 
 * The function guesses an increase in time, calculates the future position of
 * the armor, and modifies the time guess based on whether the bullet will arrive
 * too early or too late. This is done with a binary approximation algorithm
 * 
 * @param tvec translational vector of the armor
 * @param rotation armor's rotation in degrees
 * @returns The time in ms for the optimal future armor state
 */
double AntiSpintop::future_approx(cv::Mat tvec, double rotation) {
    int max_iter = 8;
    int min_error = 1;
    double time_lb = 0;
    double time_ub = 250;
    double time_mid;
    double center_coord[3] = {
        tvec.at<double>(0) - sin(rotation * M_PI / 180) * this->target_radius,
        tvec.at<double>(1),
        tvec.at<double>(2) + cos(rotation * M_PI / 180) * this->target_radius,
    };

    int iter_count = 0;

    // apply binary approximation on the time required for the bullet to hit a rotating armor
    while (time_lb < time_ub && iter_count <= max_iter) {
        iter_count++;
        time_mid = (time_lb + time_ub) / 2;
        double angle_increment = this->boxcar_average * this->spin_direction * time_mid * M_PI / 180;
        double x_center = tvec.at<double>(0) - sin(rotation * M_PI / 180) * this->target_radius;
        double z_center = tvec.at<double>(2) + cos(rotation * M_PI / 180) * this->target_radius;

        double x_new = cos(angle_increment) * (tvec.at<double>(0) - x_center)
        - sin(angle_increment) * (tvec.at<double>(2) - z_center) + x_center;
        double z_new = -sin(angle_increment) * (tvec.at<double>(0) - x_center) 
        + cos(angle_increment) * (tvec.at<double>(2) - z_center) + z_center;

        double approx_result = sqrt(x_new * x_new + z_new * z_new) / BULLET_SPEED - time_mid;

        if (abs(approx_result) < min_error) {
            break;
        }

        if (approx_result > 0) {
            time_lb = time_mid;
        } else if (approx_result < 0) {
            time_ub = time_mid;
        }
    }
    return time_mid;
}

/**
 * Estimate robot's radius with armor position and rotation
 * 
 * @param tvecs multiple armor plates' translational vector
 * @param yaws multiple armor plates' rotatoinal vector
 * @param armor_found number of armors found
 * @returns estimated radius
*/
double AntiSpintop::estimate_radius(cv::Mat* tvecs, double* yaws, int armor_found) {
    if (armor_found != 2)
    {
        return -1;
    }
    
    // trying to solve for center when seeing two armor plates
    double x_one = tvecs[0].at<double>(0);
    double z_one = tvecs[0].at<double>(2);
    double x_two = tvecs[1].at<double>(0);
    double z_two = tvecs[1].at<double>(2);

    double theta_one = -yaws[0];
    double theta_two = -yaws[1];
    double radius = (x_one - x_two) / (sin(theta_one * M_PI / 180) - sin(theta_two * M_PI / 180));

    return radius;
}

double AntiSpintop::get_yaw() {
    return this->current_yaw;
}

int AntiSpintop::get_direction() {
    return this->spin_direction;
}

double AntiSpintop::get_avg() {
    return this->boxcar_average;
}

double AntiSpintop::get_radius() {
    return this->target_radius;
}

/**
 * Calculates aim point based on current situations
 * 
 * @param tvecs multiple armor plates' translational vector
 * @param yaws multiple armor plates' rotatoinal vector
 * @param armor_found number of armors found
 * @param aim_tvec the reference of target translational vector to aim at
 * @param aim_yaw the reference of target yaw to aim at
 * @param delta_time overriding delta time (default is -1, which means 
 * the instance would automatically calculate the delta time)
*/
void AntiSpintop::calculate_aim_point(cv::Mat* tvecs, double* yaws, int armor_found, cv::Mat& aim_tvec, double& aim_yaw, int delta_time) {
    if (delta_time == -1) {
        this->delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->last_update_time).count();
    } else {
        this->delta_time = delta_time;
    }
    
    double theta_array[2] = {-yaws[0], -yaws[1]};
    double radius_est = this->estimate_radius(tvecs, yaws, armor_found);
    this->spin_direction = this->check_spinning(theta_array[0], this->slope_threshold);

    // if radius is successfully calculated
    if (radius_est != -1)
    {
        this->target_radius = fmin(fmax(radius_est, 0), MAX_ROBOT_RADIUS);
    }

    // return the targeting armor position if the robot is not spintopping
    if (this->spin_direction == 0) {
        aim_tvec = tvecs[0];
        aim_yaw = yaws[0];
    } else {
        short targeted_armor = (this->spin_direction + 1) / 2;
        double optimal_times[2] = {
            this->future_approx(tvecs[0], theta_array[0]),
            this->future_approx(tvecs[1], theta_array[1])
        };

        double angle_increment = 0;

        // two armors in sight
        if (this->armor_detected[0] && this->armor_detected[1]) {
            double delta_preceding = this->boxcar_average * this->spin_direction * optimal_times[1 - (this->spin_direction + 1) / 2] * M_PI / 180;
            double delta_next = this->boxcar_average * this->spin_direction * optimal_times[(this->spin_direction + 1) / 2] * M_PI / 180;
            
            // if optimal aiming rotation of the preceding armor exceeds the giveup threshold
            if (abs(theta_array[targeted_armor] + delta_next * 180 / M_PI) > GIVEUP_ROTATION_ABS) {
                targeted_armor = 1 - (this->spin_direction + 1) / 2;

                // if optimal aiming rotation of the next armor exceeds the giveup threshold
                if (abs(theta_array[targeted_armor] + delta_preceding * 180 / M_PI) > GIVEUP_ROTATION_ABS) {

                    // recover to the center
                    aim_tvec = (cv::Mat_<double>(3, 1) <<
                        tvecs[targeted_armor].at<double>(0) - sin(theta_array[targeted_armor] * M_PI / 180) * this->target_radius,
                        tvecs[targeted_armor].at<double>(1),
                        tvecs[targeted_armor].at<double>(2) + cos(theta_array[targeted_armor] * M_PI / 180) * this->target_radius);

                    this->last_update_time = std::chrono::steady_clock::now();
                    return;
                }
                angle_increment = delta_preceding;
            } else {
                angle_increment = delta_next;
            }
        } else if (this->armor_detected[0]) {
            targeted_armor = 0;
            angle_increment = this->boxcar_average * this->spin_direction * optimal_times[0] * M_PI / 180;
            
            if (abs(theta_array[0] + angle_increment * 180 / M_PI) > GIVEUP_ROTATION_ABS) {
                // recover to the center
                aim_tvec = (cv::Mat_<double>(3, 1) <<
                    tvecs[targeted_armor].at<double>(0) - sin(theta_array[targeted_armor] * M_PI / 180) * this->target_radius,
                    tvecs[targeted_armor].at<double>(1),
                    tvecs[targeted_armor].at<double>(2) + cos(theta_array[targeted_armor] * M_PI / 180) * this->target_radius);


                this->last_update_time = std::chrono::steady_clock::now();
                return;
            }       
        }
        

        // calculates the rotational center
        cv::Vec3d center_tvec;
        center_tvec[0] = tvecs[targeted_armor].at<double>(0) - sin(theta_array[targeted_armor] * M_PI / 180) * this->target_radius;
        center_tvec[1] = tvecs[targeted_armor].at<double>(1);
        center_tvec[2] = tvecs[targeted_armor].at<double>(2) + cos(theta_array[targeted_armor] * M_PI / 180) * this->target_radius;

        // calculates optimal future position to aim
        aim_tvec = (cv::Mat_<double>(3, 1) << 
            cos(angle_increment) * (tvecs[targeted_armor].at<double>(0) - center_tvec[0])
            - sin(angle_increment) * (tvecs[targeted_armor].at<double>(2) - center_tvec[2]) + center_tvec[0],

            center_tvec[1],

            -sin(angle_increment) * (tvecs[targeted_armor].at<double>(0) - center_tvec[0]) 
            + cos(angle_increment) * (tvecs[targeted_armor].at<double>(2) - center_tvec[2]) + center_tvec[2]);

        aim_yaw = yaws[targeted_armor] + angle_increment;
    }


    // this->last_update_time = std::chrono::steady_clock::now();
    return;
}