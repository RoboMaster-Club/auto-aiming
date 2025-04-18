#pragma once
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>

class TargetKalmanFilter
{
    public:
        TargetKalmanFilter();
        TargetKalmanFilter(float q_pos, float q_vel, float r_pos, float initial_p = 1e3f);
        ~TargetKalmanFilter();
        void configure();
        void init(float x, float y);
        bool isInitialized();
        cv::Point2f predict();
        void correct(float meas_x, float meas_y);
    private:
        float q_pos, q_vel, r_pos; 
        float initial_p = 1e3f;
        cv::KalmanFilter kf_;
        bool initialized_;
}