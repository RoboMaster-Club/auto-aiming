#include <stdio.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include "OpenCVArmorDetector.h"

void showDebugWindow(cv::Mat frame, std::vector<cv::Point2f> image_points);
std::vector<cv::Point2f> reproject_points(double theta_x, cv::Mat_<double> tvec);
cv::Mat rotation_matrix(double theta_x, double theta_y, double theta_z);
double gradient_wrt_yaw_finitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec);
double compute_loss(double theta_x, std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec);
double optimize_yaw(double theta_x, std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec);
cv::Mat rot2euler(const cv::Mat &rotationMatrix);

using Eigen::VectorXd;
using namespace LBFGSpp;

void OpenCVArmorDetector::setConfig(DetectorConfig config)
{
    _config = config;

    // Set the color limits based on config
    _blue_lower_limit = cv::Scalar(105 - _config._hue_range_limit, _config._saturation_lower_limit, _config._value_lower_limit);
    _blue_upper_limit = cv::Scalar(105 + _config._hue_range_limit, 255, 255);
    _red_lower_limit_1 = cv::Scalar(0, _config._saturation_lower_limit, _config._value_lower_limit);
    _red_upper_limit_1 = cv::Scalar(_config._hue_range_limit, 255, 255);
    _red_lower_limit_2 = cv::Scalar(179 - _config._hue_range_limit, _config._saturation_lower_limit, _config._value_lower_limit);
    _red_upper_limit_2 = cv::Scalar(179, 255, 255);

    // Set the other config variables
    _targetColor = config._target_color;
    _max_missed_frames = config._max_missed_frames;
    _reduce_search_area = config._reduce_search_area;
}

std::vector<_Float32> OpenCVArmorDetector::search(cv::Mat &frame)
{
    std::vector<_Float32> detected_keypoints(8, 0);
    static auto last_time = std::chrono::steady_clock::now(); // Static to persist across calls

    if (_reset_search_area)
    {
        _search_area[0] = 0;
        _search_area[1] = 0;
        _search_area[2] = WIDTH;
        _search_area[3] = HEIGHT;
        _reset_search_area = false;
        _missed_frames = 0;
    }
    cv::Mat croppedFrame = frame(cv::Range(_search_area[1], _search_area[3]), cv::Range(_search_area[0], _search_area[2])).clone();

    // Detect the armor in the cropped frame
    std::vector<cv::Point2f> points = detectArmorsInFrame(croppedFrame);

    // Print FPS every 500 frames
    if (_frame_count % 500 == 0 && _frame_count != 0)
    {
        // Calculate and print FPS
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
        last_time = current_time;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " [*] Detecting Armor (%d) FPS: %.2f", _frame_count, 500.0 / (elapsed_time / 1000.0));
    }
    _frame_count++;

// Display the cropped frame for debugging
#ifdef DEBUG
    cv::resize(croppedFrame, croppedFrame, cv::Size(WIDTH / 2, HEIGHT / 2));

    // Create a static window name
    const std::string window_name = "Detection Results";
    cv::imshow(window_name, croppedFrame);

    // Update the window title (OpenCV >= 4.5)
    cv::setWindowTitle(window_name,
                       "detected: " + std::to_string(_detected_frame) + " / " +
                           std::to_string(_frame_count) + " (" +
                           std::to_string(_detected_frame * 100 / _frame_count) + "%) and missed: " + std::to_string(_missed_frames) + std::string(" frames"));

    cv::waitKey(30);
#endif

    // If we didn't find an armor for a few frames (ROS2 param), reset the search area
    if (points.size() == 0)
    {
        _missed_frames++;
        if (_missed_frames >= _max_missed_frames)
        {
            _reset_search_area = true;
        }
    }
    else
    {
        // We found an armor, so reset the missed frames and return the keypoints
        _missed_frames = 0;
        std::vector<cv::Point2f> image_points;
        for (int i = 0; i < 4; i++)
        {
            detected_keypoints[i * 2] = points.at(i).x + _search_area[0];
            detected_keypoints[i * 2 + 1] = points.at(i).y + _search_area[1];

            image_points.emplace_back(cv::Point2f(points.at(i).x + _search_area[0], points.at(i).y + _search_area[1]));
        }

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " [*] Detected Armor: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)", image_points.at(0).x, image_points.at(0).y, image_points.at(1).x, image_points.at(1).y, image_points.at(2).x, image_points.at(2).y, image_points.at(3).x, image_points.at(3).y);

        if (_reduce_search_area)
        {
            // Change the search area to the bounding box of the armor with a 50 pixel buffer
            _reset_search_area = false; // We got a detection, so don't reset the search area next frame
            int x_min = (int)std::min(detected_keypoints[0], detected_keypoints[2]);
            int x_max = (int)std::max(detected_keypoints[4], detected_keypoints[6]);
            int y_min = (int)std::min(detected_keypoints[1], detected_keypoints[5]);
            int y_max = (int)std::max(detected_keypoints[3], detected_keypoints[7]);
            _search_area[0] = std::max(x_min - 50, 0);
            _search_area[1] = std::max(y_min - 50, 0);
            _search_area[2] = std::min(x_max + 50, WIDTH);
            _search_area[3] = std::min(y_max + 50, HEIGHT);
        }
        _detected_frame++;

        showDebugWindow(croppedFrame, image_points);
    }

    return detected_keypoints;
}

/**
 * @brief Detects an armor plate in the given frame.
 *
 * We Gaussian blur the input frame and convert it to HSV color space.
 * A mask is created for the desired color ranges (blue and red) and we find contours in the mask.
 * We then fit ellipses to the contours to find candidate light bars.
 * Finally, we filter for probable light bars, and attempt to find a match forming an armor plate.
 *
 * @param frame The input frame in which to search for armor.
 * @return A vector of two cv::RotatedRect objects representing the detected armor, or an empty vector if no armor is found.
 */
std::vector<cv::Point2f> OpenCVArmorDetector::detectArmorsInFrame(cv::Mat &frame)
{
    cv::Mat hsvFrame, result;

    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

    if (_targetColor == BLUE)
    {
        // Mask for blue color
        cv::inRange(hsvFrame, _blue_lower_limit, _blue_upper_limit, result);
    }
    else
    {
        // Create mask for both red ranges, mask separately and combine
        cv::Mat lower_red_inrange, upper_red_inrange;
        cv::inRange(hsvFrame, _red_lower_limit_1, _red_upper_limit_1, lower_red_inrange);
        cv::inRange(hsvFrame, _red_lower_limit_2, _red_upper_limit_2, upper_red_inrange);
        cv::bitwise_or(lower_red_inrange, upper_red_inrange, result);
    }

    // Find contours in the masked image
    std::vector<std::vector<cv::Point>> contours;
    cv::Canny(result, result, 30.0, 90.0, 3, false);
    cv::findContours(result, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // We find the light bar candidates among the contours
    std::vector<cv::RotatedRect> light_bar_candidates;
    for (auto contour : contours)
    {
        if (contour.size() > 20)
        {
            // Use convex hull to get a convex contour
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);

            auto rect_bounding = cv::boundingRect(hull);
            cv::RotatedRect rect = cv::RotatedRect(cv::Point2f(rect_bounding.x + rect_bounding.width / 2, rect_bounding.y + rect_bounding.height / 2), cv::Size2f(rect_bounding.width, rect_bounding.height), 0);

            // draw rotated rectangle
            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++)
            {
                cv::line(result, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
            }

            // Rect rotates, so we need to ensure height is always the longer side
            if (rect.angle > 45)
            {
                std::swap(rect.size.width, rect.size.height);
                rect.angle -= 90;
            }

            // cv::RotatedRect rect = cv::fitEllipse(contour);

            if (isLightBar(rect))
            {
                light_bar_candidates.push_back(rect);
            }
        }
    }

    // Give priority to the light bar with the leftmost center
    // TODO: Have a better metric such as distance from last detected armor
    std::sort(light_bar_candidates.begin(), light_bar_candidates.end(), [](cv::RotatedRect &a, cv::RotatedRect &b)
              { return a.center.x < b.center.x; });

    // If we have at least 2 light bars, we can attempt to match them to find an armor
    if (light_bar_candidates.size() >= 2)
    {
        for (int i = 0; i < light_bar_candidates.size() - 1; i++)
        {
            for (int j = i + 1; j < light_bar_candidates.size(); j++)
            {
                cv::RotatedRect rect1 = light_bar_candidates[i];
                cv::RotatedRect rect2 = light_bar_candidates[j];

                // Check if the pair of light bars likely form an armor plate
                if (isArmor(rect1, rect2))
                {
                    // We have found a match, return the pair (sorted left, right)
                    auto &first = (rect1.center.x < rect2.center.x) ? rect1 : rect2;
                    auto &second = (rect1.center.x < rect2.center.x) ? rect2 : rect1;

                    std::vector<cv::Point2f> armor_points_1 = rectToPoint(first);
                    std::vector<cv::Point2f> armor_points_2 = rectToPoint(second);

                    // Draw a dot on the top and bottom of each light bar using rectToPoint
                    cv::circle(frame, armor_points_1[0], 0, cv::Scalar(0, 255, 0), -1);
                    cv::circle(frame, armor_points_1[1], 0, cv::Scalar(0, 255, 0), -1);
                    cv::circle(frame, armor_points_2[0], 0, cv::Scalar(0, 255, 0), -1);
                    cv::circle(frame, armor_points_2[1], 0, cv::Scalar(0, 255, 0), -1);

                    return {rectToPoint(first)[0], rectToPoint(first)[1], rectToPoint(second)[0], rectToPoint(second)[1]};
                }
            }
        }
    }
    return {}; // no armor found
}

void OpenCVArmorDetector::drawRotatedRect(cv::Mat &frame, cv::RotatedRect &rect)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
}

/**
 * @brief Checks if the given RotatedRect is likely to be a light bar.
 *
 * We check the width, height, angle, and aspect ratio of the RotatedRect.
 *
 * @param rect The RotatedRect to check.
 * @return true if the RotatedRect is likely to be a light bar, false otherwise.
 */
bool OpenCVArmorDetector::isLightBar(cv::RotatedRect &rect)
{
    if (rect.size.width < LIGHT_BAR_WIDTH_LOWER_LIMIT)
    {
        return false;
    }

    if (rect.size.height < LIGHT_BAR_HEIGHT_LOWER_LIMIT)
    {
        return false;
    }

    if (rect.angle < 180 - LIGHT_BAR_ANGLE_LIMIT && rect.angle > LIGHT_BAR_ANGLE_LIMIT)
    {
        return false;
    }

    // Note we check height/width not width/height (standard aspect ratio is width/height)
    if (rect.size.height / rect.size.width < LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT)
    {
        return false;
    }
    return true;
}

/**
 * @brief Checks if a pair of given RotatedRects are likely to form an armor plate.
 *
 * We check the angle difference, aspect ratio ratio, y position ratio, height ratio, and aspect ratio of the two RotatedRects.
 *
 * @param left_rect The left RotatedRect.
 * @param right_rect The right RotatedRect.
 * @return true if the pair of RotatedRects are likely to form an armor plate, false otherwise.
 */
bool OpenCVArmorDetector::isArmor(cv::RotatedRect &left_rect, cv::RotatedRect &right_rect)
{
    float angle_diff = std::abs(left_rect.angle - right_rect.angle);

    // Light Bar Parallel Check
    if (angle_diff > ARMOR_ANGLE_DIFF_LIMIT && angle_diff < 180 - ARMOR_ANGLE_DIFF_LIMIT)
    {
        return false;
    }

    // Aspect Ratio Ratio Difference Check
    float ar_left = left_rect.size.height / left_rect.size.width;
    float ar_right = right_rect.size.height / right_rect.size.width;
    if (ar_left / ar_right > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT ||
        ar_right / ar_left > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT)
    {
        return false;
    }

    // Distance Check
    float distance = cv::norm(left_rect.center - right_rect.center);
    if (distance < left_rect.size.height || distance > 5 * left_rect.size.height)
    {
        return false;
    }

    // X Position distance Check
    if (std::abs(left_rect.center.x - right_rect.center.x) < left_rect.size.width)
    {
        return false;
    }

    // Light Bar Y Position Ratio Check
    float avg_height = (left_rect.size.height + right_rect.size.height) / 2;
    if (std::abs(left_rect.center.y - right_rect.center.y) / avg_height > ARMOR_Y_DIFF_LIMIT)
    {
        return false;
    }

    // Height Ratio Check
    if (left_rect.size.height / right_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT ||
        right_rect.size.height / left_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT)
    {
        return false;
    }

    // Aspect Ratio Check
    float max_h = std::max(left_rect.size.height, right_rect.size.height);
    float w = cv::norm(left_rect.center - right_rect.center);
    if (w / max_h > ARMOR_ASPECT_RATIO_LIMIT)
    {
        return false;
    }

    return true;
}

/**
 * @brief Converts a RotatedRect to a vector of points.
 *
 * @param rect The RotatedRect to convert.
 * @return A vector of points representing the RotatedRect.
 */
std::vector<cv::Point2f> OpenCVArmorDetector::rectToPoint(cv::RotatedRect &rect)
{
    float rad = rect.angle < 90 ? rect.angle * M_PI / 180.f : (rect.angle - 180) * M_PI / 180.f;
    float x_offset = rect.size.height * std::sin(rad) / 2.f;
    float y_offset = rect.size.height * std::cos(rad) / 2.f;

    std::vector<cv::Point2f> points;
    points = std::vector<cv::Point2f>();
    points.push_back(cv::Point2f(int(rect.center.x + x_offset), int(rect.center.y - y_offset)));
    points.push_back(cv::Point2f(int(rect.center.x - x_offset), int(rect.center.y + y_offset)));
    return points;
}

void showDebugWindow(cv::Mat frame, std::vector<cv::Point2f> image_points)
{
    float camera_matrix_[3][3];
    camera_matrix_[0][0] = 1019.108731;
    camera_matrix_[0][1] = 0;
    camera_matrix_[0][2] = 601.884969;
    camera_matrix_[1][0] = 0;
    camera_matrix_[1][1] = 1016.784980;
    camera_matrix_[1][2] = 521.004587;
    camera_matrix_[2][0] = 0;
    camera_matrix_[2][1] = 0;
    camera_matrix_[2][2] = 1;
    auto camera_matrix = cv::Mat(3, 3, CV_32F, &camera_matrix_);

    auto distortion_coefficient = cv::Mat(4, 1, cv::DataType<double>::type);
    distortion_coefficient.at<double>(0, 0) = -0.108767;
    distortion_coefficient.at<double>(1, 0) = -0.072085;
    distortion_coefficient.at<double>(2, 0) = -0.000847;
    distortion_coefficient.at<double>(3, 0) = 0.0;

    auto rvec = cv::Mat(3, 1, CV_32F);
    auto tvec = cv::Mat(3, 1, CV_32F);

    double SMALL_ARMOR_HALF_WIDTH = 135 / 2.0;
    double LIGHTBAR_HALF_HEIGHT = 55 / 2.0;

    std::vector<cv::Point3f> small_armor_object_points;
    small_armor_object_points.emplace_back(cv::Point3f(-SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0)); // Top Left
    small_armor_object_points.emplace_back(cv::Point3f(-SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0));  // Bot Left
    small_armor_object_points.emplace_back(cv::Point3f(SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0));  // Top Right
    small_armor_object_points.emplace_back(cv::Point3f(SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0));   // Bot Right
    bool PNP_OUT = cv::solvePnP(small_armor_object_points, image_points, camera_matrix, distortion_coefficient, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(small_armor_object_points, rvec, tvec, camera_matrix, distortion_coefficient, reprojected_points);
    float error = 0;
    for (int i = 0; i < reprojected_points.size(); i++)
    {
        error += cv::norm(image_points[i] - reprojected_points[i]);
    }

    // Draw img and reprojected points centered at 400, 400
    // cv::Mat reprojected_image = cv::Mat::zeros(400, 400, CV_8UC3);
    // for (int i = 0; i < reprojected_points.size(); i++)
    // {
    //     cv::circle(reprojected_image, cv::Point(reprojected_points[i].x - 350, reprojected_points[i].y - 500), 1, cv::Scalar(0, 255, 0), -1);
    //     cv::circle(reprojected_image, cv::Point(image_points[i].x - 350, image_points[i].y - 500), 1, cv::Scalar(0, 0, 255), -1);
    // }

    // rvec to rotation matrix
    // cv::Mat rotmat;
    // cv::Rodrigues(rvec, rotmat);

    // track last yaw
    static double optim = 0.0;

    // Guessing yaw
    // We use PROJECTED points as ground truth, not image points, since optimizing involves a projection
    // So we need to compare error to these
    optim = optimize_yaw(optim, reprojected_points, tvec, error);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " [*] Optimized Yaw: %.2f, Error: %.2f", optim * 180.0 / CV_PI, error);

    // Draw the reprojected points
    // cv::Mat reprojected_image_optim = cv::Mat::zeros(400, 400, CV_8UC3);

    // for (int i = 0; i < reprojected_points_optim.size(); i++)
    // {
    //     cv::circle(reprojected_image_optim, cv::Point(reprojected_points_optim[i].x - 350, reprojected_points_optim[i].y - 500), 1, cv::Scalar(0, 255, 0), -1);
    //     cv::circle(reprojected_image_optim, cv::Point(image_points[i].x - 350, image_points[i].y - 500), 1, cv::Scalar(0, 0, 255), -1);
    // }

    //
    // Drawing
    //

#ifdef DEBUG

    cv::Mat image = cv::Mat::zeros(800, 800, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255)); // Set background to white

    // Get Left/Right (x) and Up/Down (y) values from tvec
    float x = tvec.at<float>(0, 0); // Left/Right
    float y = tvec.at<float>(2, 0); // Forward/back

    // Translate to the center of the image
    int centerX = image.cols / 2;
    int centerY = image.rows / 2;

    // Scale the tvec values to fit into the image (adjust scale factor as needed)
    float scale = 0.45; // Adjust scale to fit the values onto the image
    int pointX = centerX + static_cast<int>(x * scale);
    int pointY = 400 + centerY - static_cast<int>(y * scale); // Y is inverted for screen coordinates

    // Length of the line
    int armor_length = 134;
    int length = armor_length * scale;

    // Compute the endpoint using the angle optim(in radians) int x1 = static_cast<int>(pointX + length * cos(optim)); // Calculate x offset
    int y1 = static_cast<int>(pointY + length * sin(optim)); // Calculate y offset (invert y)
    int x1 = static_cast<int>(pointX + length * cos(optim)); // Calculate x offset

    // Draw the line from (pointX, pointY) to (x1, y1)
    cv::line(image, cv::Point(pointX, pointY), cv::Point(x1, y1), cv::Scalar(0, 0, 255), 2); // Red line with thickness 2

    // draw perpendicular line from the center of the line
    // int x2 = static_cast<int>(pointX + length * cos(optim)); // Calculate x offset
    // int y2 = static_cast<int>(pointY + length * sin(optim)); // Calculate y offset (invert y)

    // Draw the line from (pointX, pointY) to (x2, y2)
    // cv::line(image, cv::Point(pointX, pointY), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2); // Green line with thickness 2

    cv::namedWindow("Top-Down View", cv::WINDOW_NORMAL); // Create window with initial title
    std::string window_title = "Top-Down View: " + std::to_string(optim * 180.0 / CV_PI);
    cv::setWindowTitle("Top-Down View", window_title); // Set the new title

    // Show the image
    cv::imshow("Top-Down View", image);
    // cv::imshow("Reprojected Image", reprojected_image_optim);
    cv::waitKey(1);
#endif
}

cv::Mat rot2euler(const cv::Mat &rotationMatrix)
{
    cv::Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998)
    { // singularity at north pole
        bank = 0;
        attitude = CV_PI / 2;
        heading = atan2(m02, m22);
    }
    else if (m10 < -0.998)
    { // singularity at south pole
        bank = 0;
        attitude = -CV_PI / 2;
        heading = atan2(m02, m22);
    }
    else
    {
        bank = atan2(-m12, m11);
        attitude = asin(m10);
        heading = atan2(-m20, m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}

//
// YAW OPTIMIZER
//

double gradient_wrt_yaw_finitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec)
{
    // Loss is strictly convex in yaw, so finite difference is a good approximation to the gradient
    double h = 1e-4;
    double loss_plus = compute_loss(yaw + h, image_points, tvec);
    double loss_minus = compute_loss(yaw - h, image_points, tvec);
    double grad = (loss_plus - loss_minus) / (2 * h);
    return grad;
}

class LossFunction
{
public:
    LossFunction(std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec) : image_points(image_points), tvec(tvec) {}

    double operator()(const VectorXd &x, VectorXd &grad)
    {
        double yaw = x(0);
        double loss = compute_loss(yaw, image_points, tvec);
        grad(0) = gradient_wrt_yaw_finitediff(yaw, image_points, tvec);
        return loss;
    }

private:
    std::vector<cv::Point2f> image_points;
    cv::Mat_<double> tvec;
};

double optimize_yaw(double initial_guess, std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec)
{
    // Optimize yaw using LBFGS Boxed
    const int n = 1;

    LBFGSBParam<double> param;
    param.epsilon = 0.4;
    param.max_iterations = 100;

    LBFGSBSolver<double, LineSearchBacktracking> solver(param);
    LossFunction loss(image_points, tvec);

    // Bounds
    VectorXd lb = VectorXd::Constant(n, -CV_PI / 4);
    VectorXd ub = VectorXd::Constant(n, CV_PI / 4);
    VectorXd x = VectorXd::Constant(n, initial_guess);

    double fx;
    int niter = solver.minimize(loss, x, fx, lb, ub);

    return x(0);
}

double compute_loss(double yaw, std::vector<cv::Point2f> image_points, cv::Mat_<double> tvec)
{
    std::vector<cv::Point2f> projected_points = reproject_points(yaw, tvec);

    double loss = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i)
    {
        double dx = projected_points[i].x - image_points[i].x;
        double dy = projected_points[i].y - image_points[i].y;
        loss += 0.25 * dx * dx + dy * dy; // Squared reprojection error
    }

    return pow(loss, 0.5);
}

std::vector<cv::Point2f> reproject_points(double yaw, cv::Mat_<double> tvec)
{
    // Camera intrinsics
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1019.109, 0, 601.885,
                             0, 1016.785, 521.005,
                             0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 4) << -0.1088, -0.0721, -0.000847, 0.0);

    // 3D object points (measured armor dimensions)
    const double SMALL_ARMOR = 134.0 / 2.0;
    const double LIGHTBAR = 54.0 / 2.0;
    std::vector<cv::Point3f> object_points = {
        {-SMALL_ARMOR, -LIGHTBAR, 0}, // Top Left
        {-SMALL_ARMOR, LIGHTBAR, 0},  // Bot Left
        {SMALL_ARMOR, -LIGHTBAR, 0},  // Top Right
        {SMALL_ARMOR, LIGHTBAR, 0}    // Bot Right
    };

    // Compute rvec
    cv::Mat R = rotation_matrix(0.0 * CV_PI / 180.0, yaw, 0.0);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    // Reproject points into image space
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    return projected_points;
}

cv::Mat rotation_matrix(double theta_x, double theta_y, double theta_z)
{
    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(theta_z), -sin(theta_z), 0,
                  sin(theta_z), cos(theta_z), 0,
                  0, 0, 1);
    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(theta_y), 0, sin(theta_y),
                  0, 1, 0,
                  -sin(theta_y), 0, cos(theta_y));
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                  0, cos(theta_x), -sin(theta_x),
                  0, sin(theta_x), cos(theta_x));
    cv::Mat R = Rz * Ry * Rx;
    return R;
}
