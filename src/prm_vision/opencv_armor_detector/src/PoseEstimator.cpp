#include "PoseEstimator.h"

void PoseEstimator::estimateTranslation(const std::vector<cv::Point2f> &image_points, bool largeArmor, cv::Mat &tvec, cv::Mat &rvec)
{
    // SolvePnP requires 3D object points
    std::vector<cv::Point3f> object_points;
    if (largeArmor)
    {
        object_points = LARGE_ARMOR_OBJECT_POINTS;
    }
    else
    {
        object_points = SMALL_ARMOR_OBJECT_POINTS;
    }

    std::string auto_aim_status;
    cv::solvePnP(object_points, image_points, CAMERA_MATRIX, DISTORTION_COEFFS, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

/**
 * @brief Estimate the yaw angle of the detected armor.
 *
 * Uses L-BFGS-B optimization to minimize the reprojection error given a guess for the yaw angle.
 * L-BFGS-B is a limited-memory quasi-Newton bounded optimization algorithm that works well for this one-dimensional problem.
 *
 * NOTE: This function assumes that pitch and roll are zero.
 * This is valid due to robot design constraints. Pitch actually is ~15 degrees, but 0 seems to work well.
 *
 * @param yaw_guess The initial guess for the yaw angle in radians.
 * @param image_points The image points of the detected armor.
 * @param tvec The translation vector of the detected armor.
 * @return double The estimated yaw angle in radians.
 */
double PoseEstimator::estimateYaw(const double yaw_guess, const std::vector<cv::Point2f> &image_points, const cv::Mat &tvec)
{
    // We only have one parameter to optimize: yaw
    const int n = 1;
    double fx;

    // L-BFGS-B parameters (experimentally tuned)
    LBFGSBParam<double> param;
    param.epsilon = 0.4;
    param.max_iterations = 50;

    LBFGSBSolver<double> solver(param);
    LossAndGradient loss(*this, image_points, tvec);

    // Bound yaw to -60 and 60 degrees
    VectorXd lb = VectorXd::Constant(n, -CV_PI / 3);
    VectorXd ub = VectorXd::Constant(n, CV_PI / 3);
    VectorXd x = VectorXd::Constant(n, yaw_guess);

    solver.minimize(loss, x, fx, lb, ub);
    return x(0);
}

/**
 * @brief Compute the gradient of the loss function with respect to yaw using finite differences.
 *
 * @param yaw The yaw angle in radians.
 * @param image_points The image points of the detected armor.
 * @param tvec The translation vector of the detected armor.
 * @return double The gradient value.
 */
double PoseEstimator::gradientWrtYawFinitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat tvec)
{
    double h = 1e-4;
    double loss_plus = computeLoss(yaw + h, image_points, tvec);
    double loss_minus = computeLoss(yaw - h, image_points, tvec);
    double grad = (loss_plus - loss_minus) / (2 * h);
    return grad;
}

/**
 * @brief Compute the loss between image points and reprojected points based on the yaw guess.
 *
 * @param yaw_guess The yaw angle guess in radians.
 * @param image_points The image points of the detected armor.
 * @param tvec The translation vector of the detected armor.
 * @return double The loss value.
 */
double PoseEstimator::computeLoss(double yaw_guess, std::vector<cv::Point2f> image_points, cv::Mat tvec)
{
    // We assume 0 pitch and roll, so they do not contribute to the rotation matrix
    cv::Mat R = (cv::Mat_<double>(3, 3) << cos(yaw_guess), 0, sin(yaw_guess),
                 0, 1, 0,
                 -sin(yaw_guess), 0, cos(yaw_guess));

    // Reproject the 3D object points using predicted yaw to the image plane
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(SMALL_ARMOR_OBJECT_POINTS, R, tvec, CAMERA_MATRIX, DISTORTION_COEFFS, projected_points);

    // Loss = sqrt(0.25 * sum((x - x')^2 + (y - y')^2))
    double loss = 0;
    for (int i = 0; i < image_points.size(); i++)
    {
        cv::Point2d diff = image_points[i] - projected_points[i];
        loss += 0.25 * (diff.x * diff.x + diff.y * diff.y); // Squared reprojection error
    }
    return pow(loss, 0.5);
}