#include "TargetKalmanFilter.hpp"



TargetKalmanFilter::TargetKalmanFilter() : initialized_(false) {

}

    // Initialize filter with given noise parameters (and optional initial state)
TargetKalmanFilter::TargetKalmanFilter(float q_pos, float q_vel, float r_pos, float initial_p = 1e3f): initialized_(false) {
        configure(q_pos, q_vel, r_pos, initial_p);
    }

    // Configure the Kalman filter matrices
void TargetKalmanFilter::configure() {
        kf_ = cv::KalmanFilter(4, 2, 0, CV_32F);
        // State: [x, y, vx, vy], Measurement: [x, y]
        // Transition matrix (F)
        // [1 0 1 0]
        // [0 1 0 1]
        // [0 0 1 0]
        // [0 0 0 1]
        kf_.transitionMatrix = (cv::Mat_<float>(4,4) << 
                                 1, 0, 1, 0,
                                 0, 1, 0, 1,
                                 0, 0, 1, 0,
                                 0, 0, 0, 1);
        // Measurement matrix (H)
        kf_.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
        kf_.measurementMatrix.at<float>(0,0) = 1.0f;
        kf_.measurementMatrix.at<float>(1,1) = 1.0f;
        // Process noise covariance (Q)
        kf_.processNoiseCov = cv::Mat::zeros(4, 4, CV_32F);
        kf_.processNoiseCov.at<float>(0,0) = this->q_pos;
        kf_.processNoiseCov.at<float>(1,1) = this->q_pos;
        kf_.processNoiseCov.at<float>(2,2) = this->q_vel;
        kf_.processNoiseCov.at<float>(3,3) = this->q_vel;
        // Measurement noise covariance (R)
        kf_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * this->r_pos;
        // Initial state error covariance (P)
        kf_.errorCovPost = cv::Mat::eye(4, 4, CV_32F) * this->initial_p;
        initialized_ = false;
    }

    // Initialize state with an observed position (x, y)
void TargetKalmanFilter::init(float x, float y) {
    kf_.statePost = (cv::Mat_<float>(4,1) << x, y, 0.0f, 0.0f);
    // (We start with zero velocity; if available, could initialize vx, vy from prior knowledge)
    initialized_ = true;
    }

bool TargetKalmanFilter::isInitialized() const { return initialized_; }

    // Predict the next state (and return predicted position)
cv::Point2f TargetKalmanFilter::predict() {
    cv::Mat predState = kf_.predict();
    // Return the predicted (x, y) position
    return cv::Point2f(predState.at<float>(0), predState.at<float>(1));
    }

    // Correct the filter with an observed position measurement (x, y)
void TargetKalmanFilter::correct(float meas_x, float meas_y) {
    cv::Mat measurement = (cv::Mat_<float>(2,1) << meas_x, meas_y);
    kf_.correct(measurement);
    }




