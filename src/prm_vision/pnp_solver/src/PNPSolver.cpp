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
    
    camera_matrix = (cv::Mat_<float>(3, 3) <<
        1019.108731f,   0.0f,          601.884969f,
        0.0f,           1016.784980f,  521.004587f,
        0.0f,           0.0f,          1.0f
    );

    distortion_coefficient = (cv::Mat_<float>(4, 1) <<
        -0.108767f,
        -0.072085f,
        -0.000847f,
        0.0f
    );

    rvec = cv::Mat(3, 1, CV_32F);
    tvec = cv::Mat(3, 1, CV_32F);
}

PNPSolver::~PNPSolver()
{
}

/**
 * @brief Perform PNP solving with image points
 * 
 * @param points image points
 * @param points_size number of items in the image points vector
 * @param large_armor whether the armor is a large armor
 * @return tvec and rvec of the pnp solution 
 */
Coordinates PNPSolver::getArmorCoordinates(std::vector<float> points, int points_size, bool large_armor)
{
    if (points_size < 8 || points[0] == 0)
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

    Coordinates output;
    output.tvec = {X, Y, Z};
    output.rvec = {rvec.at<float>(0, 0), rvec.at<float>(1, 0), rvec.at<float>(2, 0)};

    return output;
}
