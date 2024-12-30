#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "LBFGSB.h" // L-BFGS-B optimization library for yaw estimation

// Classes
#include "ValidityFilter.hpp"

// Constants for armor plates
#define LIGHTBAR_HALF_HEIGHT 54.f / 2.f
#define SMALL_ARMOR_HALF_WIDTH 134.f / 2.f
#define LARGE_ARMOR_HALF_WIDTH 225.f / 2.f

using Eigen::VectorXd;
using namespace LBFGSpp;

class PoseEstimator
{
public:
    PoseEstimator() {}
    ~PoseEstimator() {}

    // Class methods
    double estimateYaw(const double yaw_guess, const std::vector<cv::Point2f> &image_points, const cv::Mat &tvec);
    void estimateTranslation(const std::vector<cv::Point2f> &image_points, bool largeArmor, cv::Mat &tvec, cv::Mat &rvec);
    bool isValid(float x, float y, float z, std::string &auto_aim_status);

private:
    // Class methods
    ValidityFilter validity_filter_ = ValidityFilter();
    double computeLoss(double yaw_guess, std::vector<cv::Point2f> image_points, cv::Mat tvec);
    double gradientWrtYawFinitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat tvec);

    // Class variables
    double prev_valid_detection_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int locked_in_frames = 0;
    int num_frames_to_fire_after = 3;
    std::chrono::time_point<std::chrono::system_clock> last_fire_time;

    /**
     * @brief Functor for the loss function and gradient computation.
     *
     * Used for the L-BFGS-B optimization library.
     *
     * @param poseEstimator Reference to the PoseEstimator instance.
     * @param image_points The ground truth image points of the detected armor.
     * @param tvec The translation vector of the detected armor.
     * @param x The input vector containing a guess for the yaw angle.
     * @return double The loss value.
     * @return VectorXd The gradient vector.
     */
    class LossAndGradient
    {
    public:
        // Constructor to accept reference to PoseEstimator instance
        LossAndGradient(PoseEstimator &poseEstimator, std::vector<cv::Point2f> image_points, cv::Mat tvec) : poseEstimator(poseEstimator), image_points(image_points), tvec(tvec) {}

        double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad)
        {
            double yaw = x(0);
            double loss = poseEstimator.computeLoss(yaw, image_points, tvec);
            grad(0) = poseEstimator.gradientWrtYawFinitediff(yaw, image_points, tvec);
            return loss;
        }

    private:
        std::vector<cv::Point2f> image_points;
        cv::Mat tvec;
        PoseEstimator &poseEstimator;
    };

    // Camera intrinsics
    const cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) << 1019.109, 0, 601.885,
                                   0, 1016.785, 521.005,
                                   0, 0, 1);
    const cv::Mat DISTORTION_COEFFS = (cv::Mat_<double>(1, 4) << -0.1088, -0.0721, -0.000847, 0.0);

    /* 3D object points (measured armor dimensions)
     * Coordinate system:
     *
     *  +y (yaw)
     *       ^
     *       |     +x (pitch)
     *       +---->
     *      /
     *     v
     *  +z (roll)
     */
    const std::vector<cv::Point3f> SMALL_ARMOR_OBJECT_POINTS = {
        {-SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0}, // Top Left
        {-SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0},  // Bot Left
        {SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0},  // Top Right
        {SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0}    // Bot Right
    };

    const std::vector<cv::Point3f> LARGE_ARMOR_OBJECT_POINTS = {
        {-LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0}, // Top Left
        {-LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0},  // Bot Left
        {LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0},  // Top Right
        {LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0}    // Bot Right
    };
};

#endif // POSE_ESTIMATOR_HPP