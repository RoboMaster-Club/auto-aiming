#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(float q_0, float p_0, float r, float x_0, float y_0, float z_0) : q_0(q_0), p_0(p_0), r(r), x_0(x_0), y_0(y_0), z_0(z_0)
{
    this->A = Eigen::Matrix<float, 6, 6>::Identity();

    this->H = Eigen::Matrix<float, 3, 6>::Zero();
    this->H(0, 0) = 1;
    this->H(1, 1) = 1;
    this->H(2, 2) = 1;

    this->Q = Eigen::Matrix<float, 6, 6>::Identity() * this->q_0;
    this->R = Eigen::Matrix<float, 3, 3>::Identity() * this->r;
    this->X_t_measure = Eigen::Matrix<float, 3, 1>::Zero();

    this->reset();
}

// q0, p0, r = process (kinematic) noise, state covariance, measurement noise
KalmanFilter::KalmanFilter() : KalmanFilter(10, 10000, 20, 0, 0, 0) {}

KalmanFilter::~KalmanFilter()
{
    return;
}

void KalmanFilter::reset()
{

    this->X = Eigen::Matrix<float, 6, 1>::Zero();
    this->X(0, 0) = this->x_0;
    this->X(1, 0) = this->y_0;
    this->X(2, 0) = this->z_0;
    this->X(3, 0) = 0.f;
    this->X(4, 0) = 0.f;
    this->X(5, 0) = 0.f;

    this->P = Eigen::Matrix<float, 6, 6>::Identity() * this->p_0;
}

void KalmanFilter::update(float pos_x, float pos_y, float pos_z, float dt, float dst[6])
{

    this->X_t_measure(0, 0) = pos_x;
    this->X_t_measure(1, 0) = pos_y;
    this->X_t_measure(2, 0) = pos_z;

    this->A(0, 3) = dt;
    this->A(1, 4) = dt;
    this->A(2, 5) = dt;

    this->X_hat_t = this->A * this->X;
    this->P_hat_t = this->A * this->P * this->A.transpose() + this->Q;

    this->z_t = Eigen::Matrix<float, 3, 3>::Identity() * this->X_t_measure;
    this->y_t = this->z_t - this->H * this->X_hat_t;

    this->s_t = this->H * this->P_hat_t * this->H.transpose() + this->R;
    this->K = this->P_hat_t * this->H.transpose() * this->s_t.inverse();

    this->P = (Eigen::Matrix<float, 6, 6>::Identity() - this->K * this->H) * this->P_hat_t;
    this->X = this->X_hat_t + this->K * this->y_t;

    dst[0] = this->X(0, 0);
    dst[1] = this->X(1, 0);
    dst[2] = this->X(2, 0);
    dst[3] = this->X(3, 0);
    dst[4] = this->X(4, 0);
    dst[5] = this->X(5, 0);
}
