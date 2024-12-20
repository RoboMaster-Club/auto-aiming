#ifndef _KALMAN_FILTER_HPP
#define _KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <iostream>

class KalmanFilter
{
public:
    KalmanFilter(float, float, float, float, float, float);

    KalmanFilter();

    ~KalmanFilter();

    void update(float, float, float, float, float[]);

    void reset();

private:


    float q_0;
    float p_0;
    float r;
    float x_0;      // in mm
    float y_0;      // in mm
    float z_0;      // in mm

    Eigen::Matrix<float, 6, 6> A;
    Eigen::Matrix<float, 3, 6> H;
    Eigen::Matrix<float, 6, 6> Q;
    Eigen::Matrix<float, 6, 6> P;
    Eigen::Matrix<float, 3, 3> R;
    Eigen::Matrix<float, 6, 1> X;

    Eigen::Matrix<float, 3, 1> X_t_measure;

    Eigen::Matrix<float, 6, 1> X_hat_t;
    Eigen::Matrix<float, 6, 6> P_hat_t;

    Eigen::Matrix<float, 3, 1> y_t;
    Eigen::Matrix<float, 3, 1> z_t;

    Eigen::Matrix<float, 3, 3> s_t;
    Eigen::Matrix<float, 6, 3> K;

    
};

#endif // _KALMAN_FILTER_HPP