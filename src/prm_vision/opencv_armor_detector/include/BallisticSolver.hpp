#pragma once
#include <cmath>
#include <map>
#include <algorithm>
#include <opencv2/core.hpp>
class BallisticSolver {
public:
    BallisticSolver();
    double computePitchAngle(const cv::Vec3d& target_pos);
    void setParameters(double speed, double gravity, bool useAnalytical);
    void BallisticSolver::setCameraOffset(double ox, double oy, double oz);
    cv::Vec3d transformCameraToGimbal(const cv::Vec3d& target_cam);
    double computePitchAngle(const cv::Vec3d& target_pos);
private:
    double projectile_speed_;  // projectile muzzle speed (m/s)
    double gravity_;           // gravity (m/s^2)
    bool use_analytical_;      // true for analytical model, false for LUT mode

    // Camera-to-gimbal offset (in meters, in camera frame axes)
    double offset_x_, offset_y_, offset_z_;
}