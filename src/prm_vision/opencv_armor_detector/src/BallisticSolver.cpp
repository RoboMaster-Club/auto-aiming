// ballistic_solver.hpp
#include "BallisticSolver.hpp"



BallisticSolver::BallisticSolver(): projectile_speed_(15.0), gravity_(9.81), use_analytical_(true) {
    
       
        // Default values: speed 15 m/s, gravity 9.81 m/s^2 (tune via parameters)
        // Default offsets (camera-to-gimbal) as zero; set via parameters if needed
        offset_x_ = offset_y_ = offset_z_ = 0.0;
    }

    // Configure ballistic parameters (could be called from ROS2 param update)
void BallisticSolver::setParameters(double speed, double gravity, bool useAnalytical) {
    projectile_speed_ = speed;
    gravity_ = gravity;
    use_analytical_ = useAnalytical;
}

void BallisticSolver::setCameraOffset(double ox, double oy, double oz) {
    offset_x_ = ox;
    offset_y_ = oy;
    offset_z_ = oz;
}

    // Transform 3D target position from camera frame to gimbal frame by applying the offset
    cv::Vec3d BallisticSolver::transformCameraToGimbal(const cv::Vec3d& target_cam) const {
        // Assuming camera and gimbal axes are aligned, just translate by the offset
        cv::Vec3d target_gimbal;
        target_gimbal[0] = target_cam[0] + offset_x_;
        target_gimbal[1] = target_cam[1] + offset_y_;
        target_gimbal[2] = target_cam[2] + offset_z_;
        return target_gimbal;
    }

    // Compute the required pitch angle (in radians) to hit the target at target_pos (in gimbal frame)
    double BallisticSolver::computePitchAngle(const cv::Vec3d& target_pos) const {
        // Calculate horizontal distance and vertical difference
        double X = target_pos[0];
        double Y = target_pos[1];
        double Z = target_pos[2];
        double horizontal_dist = std::sqrt(X*X + Z*Z);
        double vertical_diff = -Y;  // vertical difference (positive if target is above the camera)
        double v = projectile_speed_;
        double g = gravity_;

        if (!use_analytical_) {
            // Lookup table mode: use pre-defined mappings from distance to pitch
            // (For demonstration, a simple linear interpolation on an example table)
            static const std::map<double, double> pitch_table = {
                {5.0,  5.0 * M_PI/180.0},   // 5m -> 5 deg
                {10.0, 10.0 * M_PI/180.0},  // 10m -> 10 deg
                {15.0, 15.0 * M_PI/180.0}   // 15m -> 15 deg (just an example linear progression)
            };
            double dist = std::sqrt(X*X + Y*Y + Z*Z);  // straight-line distance
            auto it = pitch_table.lower_bound(dist);
            double pitch_angle = 0.0;
            if (it == pitch_table.end()) {
                pitch_angle = pitch_table.rbegin()->second;
            } else if (it == pitch_table.begin()) {
                pitch_angle = it->second;
            } else {
                // Linear interpolate between it (upper bound) and previous point
                double d2 = it->first;
                double a2 = it->second;
                auto it1 = std::prev(it);
                double d1 = it1->first;
                double a1 = it1->second;
                double t = (dist - d1) / (d2 - d1);
                pitch_angle = a1 + t * (a2 - a1);
            }
            return pitch_angle;
        }

        // Analytical mode: solve for the launch angle to hit target (assuming level shot for simplicity)
        // Using projectile motion formula for different elevation:
        // tanθ = (v^2 ± sqrt(v^4 - g*(g*horizontal_dist^2 + 2*vertical_diff*v^2))) / (g * horizontal_dist)
        double v2 = v * v;
        double term = v2*v2 - g * (g * horizontal_dist*horizontal_dist + 2 * vertical_diff * v2);
        if (term < 0) {
            // No physical solution (target out of range for given speed), clamp angle to 45 degrees
            return M_PI / 4;
        }
        double sqrt_term = std::sqrt(term);
        double tan_angle1 = (v2 - sqrt_term) / (g * horizontal_dist);
        double tan_angle2 = (v2 + sqrt_term) / (g * horizontal_dist);
        double angle1 = std::atan(tan_angle1);
        double angle2 = std::atan(tan_angle2);
        // Choose the smaller positive angle (for a direct shot rather than a high-arcing shot)
        double pitch_angle = angle1;
        if (angle2 > 0 && angle2 < angle1) {
            pitch_angle = angle2;
        }
        return pitch_angle;
    }

private:
    double projectile_speed_;  // projectile muzzle speed (m/s)
    double gravity_;           // gravity (m/s^2)
    bool use_analytical_;      // true for analytical model, false for LUT mode

    // Camera-to-gimbal offset (in meters, in camera frame axes)
    double offset_x_, offset_y_, offset_z_;


 // namespace prm_vision
