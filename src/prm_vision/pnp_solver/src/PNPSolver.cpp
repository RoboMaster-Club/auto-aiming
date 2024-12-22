#include "PNPSolver.hpp"

PNPSolver::PNPSolver()
{
    // +X: Out, +Y: Right, +Z: Up
    small_armor_object_points.emplace_back(cv::Point3f(0, -SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));  // Top Left
    small_armor_object_points.emplace_back(cv::Point3f(0, -SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT)); // Bot Left
    small_armor_object_points.emplace_back(cv::Point3f(0, SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));   // Top Right
    small_armor_object_points.emplace_back(cv::Point3f(0, SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT));  // Bot Right

    large_armor_object_points.emplace_back(cv::Point3f(0, -LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));  // Top Left
    large_armor_object_points.emplace_back(cv::Point3f(0, -LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT)); // Bot Left
    large_armor_object_points.emplace_back(cv::Point3f(0, LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT));   // Top Righy
    large_armor_object_points.emplace_back(cv::Point3f(0, LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT));  // Bot Right

    camera_matrix_[0][0] = 1019.108731;
    camera_matrix_[0][1] = 0;
    camera_matrix_[0][2] = 601.884969;
    camera_matrix_[1][0] = 0;
    camera_matrix_[1][1] = 1016.784980;
    camera_matrix_[1][2] = 521.004587;
    camera_matrix_[2][0] = 0;
    camera_matrix_[2][1] = 0;
    camera_matrix_[2][2] = 1;

    camera_matrix = cv::Mat(3, 3, CV_32F, &camera_matrix_);

    distortion_coefficient = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
    distortion_coefficient.at<double>(0, 0) = -0.108767;
    distortion_coefficient.at<double>(1, 0) = -0.072085;
    distortion_coefficient.at<double>(2, 0) = -0.000847;
    distortion_coefficient.at<double>(3, 0) = 0.0;

    rvec = cv::Mat(3, 1, CV_32F);
    tvec = cv::Mat(3, 1, CV_32F);
}

PNPSolver::~PNPSolver()
{
}


Coordinates PNPSolver::getArmorCoordinates(float points[], int points_size, bool large_armor)
{
    if (points_size != 8 || points[0] == 0)
    {
        return (Coordinates){};
    }

    std::vector<cv::Point2f> image_points;
    image_points.push_back(cv::Point2f(points[0], points[1]));
    image_points.push_back(cv::Point2f(points[2], points[3]));
    image_points.push_back(cv::Point2f(points[4], points[5]));
    image_points.push_back(cv::Point2f(points[6], points[7]));

    bool result = cv::solvePnP(small_armor_object_points, image_points, camera_matrix, distortion_coefficient, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    if (!result)
    {
        return (Coordinates){};
    }

    float Y = -tvec.at<float>(0, 0);
    float Z = -tvec.at<float>(1, 0);
    float X = tvec.at<float>(2, 0);
    tvec.at<float>(0, 0) = X;
    tvec.at<float>(1, 0) = Y;
    tvec.at<float>(2, 0) = Z;
    std::vector<float> output = {X, Y, Z};

    return (Coordinates){{X, Y, Z}, {rvec.at<float>(0, 0), rvec.at<float>(1, 0), rvec.at<float>(2, 0)}};
}
