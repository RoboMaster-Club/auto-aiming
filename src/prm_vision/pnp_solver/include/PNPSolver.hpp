#ifndef _PNP_SOLVER_HPP
#define _PNP_SOLVER_HPP

#include <fstream>
#include <math.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#define LIGHTBAR_HALF_HEIGHT 54.f / 2.f
#define SMALL_ARMOR_HALF_WIDTH 134.f / 2.f
#define LARGE_ARMOR_HALF_WIDTH 225.f / 2.f

typedef struct Coordinates {
    std::vector<float> tvec;
    std::vector<float> rvec;
} Coordinates;

class PNPSolver
{
public:
    PNPSolver();
    ~PNPSolver();

    Coordinates getArmorCoordinates(std::vector<float> points, int points_size, bool large_armor);
private:
    std::vector<cv::Point3f> small_armor_object_points;
    std::vector<cv::Point3f> large_armor_object_points;
    cv::Mat distortion_coefficient;
    cv::Mat camera_matrix;

    cv::Mat rvec;
    cv::Mat tvec;
};

#endif // _PNP_SOLVER_HPP
