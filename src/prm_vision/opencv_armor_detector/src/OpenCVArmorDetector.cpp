#include <stdio.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include "OpenCVArmorDetector.h"

/*Color Filtering and Purple-Light Rejection
In this section, we implement HSV color thresholding to detect target team colors (blue or red) while masking out purple light artifacts. The filterColor() method converts the input image to HSV and applies color range filters. For blue detection, any pixels in the purple range are removed from the blue mask. (For red detection, purple overlap is minimal but we still demonstrate masking for completeness.) This can be integrated into OpenCVArmorDetectorNode or a separate ColorFilter module.*/

// color_filter.hpp (within prm_vision namespace)
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace prm_vision {

    enum TargetColor { RED = 0, BLUE = 1 };

    class ColorFilter {
    public:
        // Constructors allow configuration of HSV ranges for colors
        ColorFilter(TargetColor target)
            : target_color_(target) {
            // Default HSV ranges for blue, red, and purple (tunable via parameters if needed)
            blue_lower_ = cv::Scalar(90, 100, 100);  // lower bound for blue hue
            blue_upper_ = cv::Scalar(130, 255, 255);  // upper bound for blue hue
            purple_lower_ = cv::Scalar(130, 120, 120);  // lower bound for purple hue
            purple_upper_ = cv::Scalar(160, 255, 255);  // upper bound for purple hue
            red_lower1_ = cv::Scalar(0, 120, 120);  // red hue segment 1 (near 0)
            red_upper1_ = cv::Scalar(10, 255, 255);
            red_lower2_ = cv::Scalar(170, 120, 120);  // red hue segment 2 (near 180)
            red_upper2_ = cv::Scalar(179, 255, 255);
        }

        // Set target color at runtime (e.g., via ROS2 parameter)
        void setTargetColor(TargetColor color) {
            target_color_ = color;
        }

        // Filter the image to produce a binary mask of the target color, excluding purple
        cv::Mat apply(const cv::Mat& bgr_image) {
            cv::Mat hsv;
            cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

            cv::Mat mask_target;
            if (target_color_ == BLUE) {
                // Threshold for blue
                cv::inRange(hsv, blue_lower_, blue_upper_, mask_target);
                // Create mask for purple regions and remove them from blue mask
                cv::Mat mask_purple;
                cv::inRange(hsv, purple_lower_, purple_upper_, mask_purple);
                // Subtract purple from blue mask
                mask_target &= ~mask_purple;
            }
            else { // RED
                // Threshold for red (covering wrap-around hue)
                cv::Mat mask_red1, mask_red2;
                cv::inRange(hsv, red_lower1_, red_upper1_, mask_red1);
                cv::inRange(hsv, red_lower2_, red_upper2_, mask_red2);
                mask_target = mask_red1 | mask_red2;
                // Threshold and remove purple just in case (purple mostly affects blue, but include for completeness)
                cv::Mat mask_purple;
                cv::inRange(hsv, purple_lower_, purple_upper_, mask_purple);
                mask_target &= ~mask_purple;
            }

            // Optional: perform morphological ops to clean up noise (not strictly required, can be tuned)
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(mask_target, mask_target, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask_target, mask_target, cv::MORPH_CLOSE, kernel);
            return mask_target;
        }

    private:
        TargetColor target_color_;
        cv::Scalar blue_lower_, blue_upper_;
        cv::Scalar purple_lower_, purple_upper_;
        cv::Scalar red_lower1_, red_upper1_, red_lower2_, red_upper2_;
    };

} // namespace prm_vision






/*Hybrid Contour Fitting for Light Bars
This section improves orientation extraction of light bar contours by using ellipse fitting with a fallback to minimum-area rectangle. The fitLightContour() function tries cv::fitEllipseDirect to get an accurate angle of the elongated contour. If the fit is unreliable (e.g., contour is too circular or has too few points), it falls back to cv::minAreaRect. The orientation angle is normalized to a consistent range [0, 180) degrees for further processing.
// contour_fitting.hpp (within prm_vision namespace)*/
#pragma once
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

namespace prm_vision {

    // Fit a light-bar contour to an ellipse (if possible) or a min-area rectangle, and normalize its angle.
    static cv::RotatedRect fitLightContour(const std::vector<cv::Point>& contour) {
        cv::RotatedRect rect;
        if (contour.size() >= 5) {
            // Try fitting an ellipse to the contour
            cv::RotatedRect ellipseRect = cv::fitEllipseDirect(contour);
            // Calculate aspect ratio of the fitted ellipse
            float major = std::max(ellipseRect.size.width, ellipseRect.size.height);
            float minor = std::min(ellipseRect.size.width, ellipseRect.size.height);
            bool unstable_fit = (minor < 1e-5) || (major / minor < 1.5f);
            if (!unstable_fit) {
                rect = ellipseRect;
            }
            else {
                // Fallback to minAreaRect if ellipse is unstable (e.g., nearly circular or degenerate)
                rect = cv::minAreaRect(contour);
            }
        }
        else {
            // Not enough points for ellipse, use minAreaRect directly
            rect = cv::minAreaRect(contour);
        }

        // Normalize the orientation angle of the rectangle/ellipse:
        // OpenCV's RotatedRect::angle is in degrees and can range from -90 to 0 for minAreaRect.
        float angle = rect.angle;
        // Ensure angle corresponds to the long side of the light bar pointing direction
        if (rect.size.width < rect.size.height) {
            // If width < height, the returned angle is measured from the vertical axis
            // Add 90 degrees to align angle with the long axis (light bar orientation)
            angle += 90.0f;
        }
        // Normalize angle to [0, 180)
        if (angle < 0) angle += 180.0f;
        if (angle >= 180.0f) angle -= 180.0f;
        rect.angle = angle;
        return rect;
    }

} // namespace prm_vision





/*Detector Acceleration (ROI, Early-Exit, Multi-threading)
This section addresses performance improvements: dynamic ROI narrowing, early exit when no targets are found, and parallel processing of image capture and detection. The OpenCVArmorDetectorNode class (within prm_vision) manages the image subscription and processing thread. It uses a persistent ROI (Region of Interest) that focuses the image processing around the last known target location to reduce computation, and falls back to full-frame search if the target is lost. The processing loop exits early if no contours are found to skip unnecessary calculations.

// opencv_armor_detector_node.hpp*/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include "color_filter.hpp"
#include "contour_fitting.hpp"
#include "target_kalman_filter.hpp"
#include "ballistic_solver.hpp"

namespace prm_vision {

    class OpenCVArmorDetectorNode : public rclcpp::Node {
    public:
        OpenCVArmorDetectorNode()
            : Node("opencv_armor_detector"),
            color_filter_(TargetColor::BLUE)  // default to detect blue armors; can be changed via param
        {
            // Declare and get parameters for color target (0=RED, 1=BLUE)
            int target_color = this->declare_parameter<int>("target_color", 1);
            color_filter_.setTargetColor(target_color == 0 ? TargetColor::RED : TargetColor::BLUE);

            // Set up image subscriber (no heavy processing in callback)
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10,
                std::bind(&OpenCVArmorDetectorNode::imageCallback, this, std::placeholders::_1)
            );

            // Start a dedicated thread for image processing
            processing_thread_ = std::thread(&OpenCVArmorDetectorNode::processingLoop, this);

            RCLCPP_INFO(this->get_logger(), "OpenCVArmorDetectorNode started with target_color=%s",
                target_color == 0 ? "RED" : "BLUE");
        }

        ~OpenCVArmorDetectorNode() {
            // Signal thread to exit and join
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                running_ = false;
                frame_cv_.notify_one();
            }
            if (processing_thread_.joinable()) {
                processing_thread_.join();
            }
        }

    private:
        // Image subscriber and separate processing thread
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        std::thread processing_thread_;
        std::mutex frame_mutex_;
        std::condition_variable frame_cv_;
        cv::Mat latest_frame_;
        bool frame_ready_ = false;
        bool running_ = true;

        // Last known target ROI (in full-frame coordinates) for dynamic ROI tracking
        cv::Rect last_target_roi_;
        bool have_last_target_ = false;

        // Color filter for target detection (blue/red)
        ColorFilter color_filter_;

        // Kalman filter for target center prediction (initialized later when first target appears)
        TargetKalmanFilter kalman_filter_;

        // Ballistic solver for pitch calculation and transform offsets
        BallisticSolver ballistic_solver_;

        // Camera-to-gimbal offset (loaded via parameters in BallisticSolver or separate)
        // (Camera transform handled in ballistic_solver_ or could be separate function)

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            // Convert ROS image to OpenCV BGR matrix
            try {
                cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
                // Lock and update the latest frame
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    latest_frame_ = frame;
                    frame_ready_ = true;
                }
                frame_cv_.notify_one();
            }
            catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "CV Bridge conversion failed: %s", e.what());
            }
        }

        void processingLoop() {
            rclcpp::WallRate rate(120);  // target max processing rate (in Hz) if frames arrive faster
            while (rclcpp::ok() && running_) {
                cv::Mat frame;
                // Wait for a new frame from the camera callback
                {
                    std::unique_lock<std::mutex> lock(frame_mutex_);
                    frame_cv_.wait(lock, [this]() { return frame_ready_ || !running_; });
                    if (!running_) break;  // exit signal
                    frame = latest_frame_.clone();
                    frame_ready_ = false;
                }

                // Process the frame (run detection pipeline)
                processFrame(frame);

                rate.sleep();
            }
        }

        void processFrame(const cv::Mat& frame) {
            cv::Mat image = frame;
            cv::Rect search_roi;
            if (have_last_target_) {
                // Define a search ROI around last known target (expanded by 20% in each direction)
                search_roi = last_target_roi_;
                // Expand ROI by 20% (to account for motion) and clamp to image bounds
                int dx = search_roi.width / 5;
                int dy = search_roi.height / 5;
                search_roi.x = std::max(0, search_roi.x - dx);
                search_roi.y = std::max(0, search_roi.y - dy);
                search_roi.width = std::min(image.cols - search_roi.x, search_roi.width + 2 * dx);
                search_roi.height = std::min(image.rows - search_roi.y, search_roi.height + 2 * dy);
                // Crop the image to the ROI for faster processing
                image = frame(search_roi);
                RCLCPP_DEBUG(this->get_logger(), "Using ROI (%d,%d,%d,%d) for detection",
                    search_roi.x, search_roi.y, search_roi.width, search_roi.height);
            }

            // Apply color filter to get binary mask of potential light bars
            cv::Mat mask = color_filter_.apply(image);

            // Find contours of the light bars in the mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (contours.empty()) {
                // No contours found – if we were tracking a target, this might indicate target lost
                if (have_last_target_) {
                    RCLCPP_DEBUG(this->get_logger(), "No contours in ROI, trying full frame...");
                    // If ROI search failed, try full frame once before declaring loss
                    have_last_target_ = false;  // next loop will do full-frame search
                    // (Alternatively, we could attempt full-frame immediately here by continuing detection, but for simplicity, we mark lost and exit this frame)
                }
                // Skip further processing for this frame
                return;
            }

            // Analyze contours to find light bar orientations
            struct LightBar { cv::RotatedRect rect; };
            std::vector<LightBar> light_bars;
            light_bars.reserve(contours.size());
            for (auto& contour : contours) {
                if (contour.size() < 5 || cv::contourArea(contour) < 10.0) {
                    // Skip very small contours (noise)
                    continue;
                }
                cv::RotatedRect rrect = fitLightContour(contour);
                // Offset the rectangle coordinates if ROI was used (to map back to full frame)
                if (have_last_target_) {
                    rrect.center.x += search_roi.x;
                    rrect.center.y += search_roi.y;
                }
                light_bars.push_back({ rrect });
            }

            if (light_bars.empty()) {
                // No valid light bars found
                RCLCPP_DEBUG(this->get_logger(), "Contours found, but none valid as light bars.");
                have_last_target_ = false;
                return;
            }

            // If multiple light bars, select or pair them into an armor target.
            // For simplicity, here we assume one armor target and pick the best two bars:
            // (In practice, you'd pair bars by size/angle; omitted for brevity)
            cv::RotatedRect left_bar = light_bars[0].rect;
            cv::RotatedRect right_bar = light_bars.size() > 1 ? light_bars[1].rect : light_bars[0].rect;
            if (right_bar.center.x < left_bar.center.x) std::swap(left_bar, right_bar);

            // Compute approximate armor ROI and corners based on the two bars
            cv::Point2f leftCorners[4], rightCorners[4];
            left_bar.points(leftCorners);
            right_bar.points(rightCorners);
            // Sort corners by y (to identify top and bottom points)
            std::sort(leftCorners, leftCorners + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
            std::sort(rightCorners, rightCorners + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
            // Define armor corners: top-left, top-right, bottom-right, bottom-left
            std::vector<cv::Point2f> imagePoints = {
                leftCorners[0],  // top-left
                rightCorners[0], // top-right
                rightCorners[3], // bottom-right
                leftCorners[3]   // bottom-left
            };

            // Update last_target_roi_ for next frame (bounding box of the armor)
            last_target_roi_ = cv::boundingRect(imagePoints);
            have_last_target_ = true;

            // Prepare 3D object points of the armor (assuming known real dimensions, e.g., 130mm x 55mm armor plate)
            double armor_width = 0.130;   // 130 mm in meters
            double armor_height = 0.055;  // 55 mm in meters
            std::vector<cv::Point3f> objectPoints = {
                {-armor_width / 2, -armor_height / 2, 0.0},  // top-left corner in object frame
                { armor_width / 2, -armor_height / 2, 0.0},  // top-right
                { armor_width / 2,  armor_height / 2, 0.0},  // bottom-right
                {-armor_width / 2,  armor_height / 2, 0.0}   // bottom-left
            };

            // Camera intrinsics (should be set from calibration parameters; here assume fetched or known)
            static cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
                /*fx*/ 920.0, 0.0,    /*cx*/ 640.0,
                0.0, 920.0, /*cy*/ 360.0,
                0.0, 0.0, 1.0);
            static cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);  // assume no distortion or pre-rectified

            // Solve PnP with IPPE to get two possible poses
            std::vector<cv::Vec3d> rvecs, tvecs;
            bool pnp_ok = cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE);
            if (!pnp_ok || tvecs.empty()) {
                RCLCPP_WARN(this->get_logger(), "PnP pose estimation failed.");
                return;
            }

            // Choose the correct pose by reprojection error if two solutions found
            cv::Vec3d best_rvec = rvecs[0], best_tvec = tvecs[0];
            if (tvecs.size() > 1) {
                double minError = std::numeric_limits<double>::max();
                for (size_t i = 0; i < tvecs.size(); ++i) {
                    // Project object points back onto image for each solution
                    std::vector<cv::Point2f> reprojected;
                    cv::projectPoints(objectPoints, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, reprojected);
                    // Compute total reprojection error
                    double error = 0.0;
                    for (size_t j = 0; j < reprojected.size(); ++j) {
                        error += cv::norm(reprojected[j] - imagePoints[j]);
                    }
                    if (error < minError) {
                        minError = error;
                        best_rvec = rvecs[i];
                        best_tvec = tvecs[i];
                    }
                }
            }

            // Apply camera-to-gimbal transform to the 3D target position (shifting by camera offset)
            cv::Vec3d target_pos_cam = best_tvec;  // target position in camera coordinates
            cv::Vec3d target_pos_gimbal = ballistic_solver_.transformCameraToGimbal(target_pos_cam);

            // Extract yaw angle to target (in gimbal frame)
            double X = target_pos_gimbal[0];
            double Y = target_pos_gimbal[1];
            double Z = target_pos_gimbal[2];
            double yaw_rad = std::atan2(X, Z);
            double yaw_deg = yaw_rad * 180.0 / M_PI;

            // Use ballistic model to compute required pitch angle
            double pitch_rad = ballistic_solver_.computePitchAngle(target_pos_gimbal);
            double pitch_deg = pitch_rad * 180.0 / M_PI;

            // (Optionally) Use Kalman filter to smooth the target angles or position
            // Here we feed the center-of-armor pixel position to the Kalman filter for smoothing and prediction.
            cv::Point2f armor_center = 0.5f * (left_bar.center + right_bar.center);
            if (!kalman_filter_.isInitialized()) {
                kalman_filter_.init(armor_center.x, armor_center.y);
            }
            cv::Point2f prediction = kalman_filter_.predict();  // predict next position (for next frame ROI)
            kalman_filter_.correct(armor_center.x, armor_center.y);

            // Log or publish the results (yaw, pitch)
            RCLCPP_INFO(this->get_logger(), "Target found. Yaw=%.2f deg, Pitch=%.2f deg, Dist=%.2f m",
                yaw_deg, pitch_deg, cv::norm(target_pos_gimbal));
            // (In a real system, publish the yaw and pitch or full pose here for the gimbal controller)
        }
    };

} // namespace prm_vision










/*Kalman Filter Module for Target Tracking
We introduce a TargetKalmanFilter class to provide predictive tracking of the target’s position (in image pixel coordinates or angles) between frames. This helps smooth out noise and can aid the ROI selection by predicting where the target will appear next. The filter is a constant-velocity model in 2D (x, y) with state vector [x, y, vx, vy]. The class allows configuration of the process noise covariance Q, measurement noise covariance R, and initial error covariance P via ROS2 parameters or constructor arguments.


// target_kalman_filter.hpp*/
#pragma once
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>

namespace prm_vision {

    class TargetKalmanFilter {
    public:
        TargetKalmanFilter()
            : initialized_(false) {}

        // Initialize filter with given noise parameters (and optional initial state)
        TargetKalmanFilter(float q_pos, float q_vel, float r_pos, float initial_p = 1e3f)
            : initialized_(false) {
            configure(q_pos, q_vel, r_pos, initial_p);
        }

        // Configure the Kalman filter matrices
        void configure(float q_pos, float q_vel, float r_pos, float initial_p = 1e3f) {
            kf_ = cv::KalmanFilter(4, 2, 0, CV_32F);
            // State: [x, y, vx, vy], Measurement: [x, y]
            // Transition matrix (F)
            // [1 0 1 0]
            // [0 1 0 1]
            // [0 0 1 0]
            // [0 0 0 1]
            kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1);
            // Measurement matrix (H)
            kf_.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
            kf_.measurementMatrix.at<float>(0, 0) = 1.0f;
            kf_.measurementMatrix.at<float>(1, 1) = 1.0f;
            // Process noise covariance (Q)
            kf_.processNoiseCov = cv::Mat::zeros(4, 4, CV_32F);
            kf_.processNoiseCov.at<float>(0, 0) = q_pos;
            kf_.processNoiseCov.at<float>(1, 1) = q_pos;
            kf_.processNoiseCov.at<float>(2, 2) = q_vel;
            kf_.processNoiseCov.at<float>(3, 3) = q_vel;
            // Measurement noise covariance (R)
            kf_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * r_pos;
            // Initial state error covariance (P)
            kf_.errorCovPost = cv::Mat::eye(4, 4, CV_32F) * initial_p;
            initialized_ = false;
        }

        // Initialize state with an observed position (x, y)
        void init(float x, float y) {
            kf_.statePost = (cv::Mat_<float>(4, 1) << x, y, 0.0f, 0.0f);
            // (We start with zero velocity; if available, could initialize vx, vy from prior knowledge)
            initialized_ = true;
        }

        bool isInitialized() const { return initialized_; }

        // Predict the next state (and return predicted position)
        cv::Point2f predict() {
            cv::Mat predState = kf_.predict();
            // Return the predicted (x, y) position
            return cv::Point2f(predState.at<float>(0), predState.at<float>(1));
        }

        // Correct the filter with an observed position measurement (x, y)
        void correct(float meas_x, float meas_y) {
            cv::Mat measurement = (cv::Mat_<float>(2, 1) << meas_x, meas_y);
            kf_.correct(measurement);
        }

    private:
        cv::KalmanFilter kf_;
        bool initialized_;
    };

} // namespace prm_vision






/*Yaw Estimation and Pose Disambiguation
Once potential armor plate targets are identified, the system estimates the 3D pose using SolvePnP and extracts the yaw angle for aiming. Here we use cv::solvePnPGeneric with the SOLVEPNP_IPPE_SQUARE method to account for the square planar nature of the armor marker, which can return two possible pose solutions (mirror poses). We then perform pose disambiguation by reprojecting the 3D corners and computing the reprojection error for each solution, selecting the one with the lowest error.
(This logic is implemented in the processFrame() method of the node above, but highlighted here for clarity.)

// ... inside OpenCVArmorDetectorNode::processFrame (excerpt for pose estimation)*/
std::vector<cv::Vec3d> rvecs, tvecs;
cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, distCoeffs,
    rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE);
cv::Vec3d best_rvec, best_tvec;
if (tvecs.size() > 1) {
    // Compute reprojection error for each pose
    double bestError = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tvecs.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objectPoints, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projected);
        double error = 0.0;
        for (size_t j = 0; j < projected.size(); ++j) {
            error += cv::norm(projected[j] - imagePoints[j]);
        }
        if (error < bestError) {
            bestError = error;
            best_rvec = rvecs[i];
            best_tvec = tvecs[i];
        }
    }
}
else if (tvecs.size() == 1) {
    best_rvec = rvecs[0];
    best_tvec = tvecs[0];
}
else {
    // No solution found
    return;
}
// Compute yaw from the chosen pose (after camera-to-gimbal transform)
cv::Vec3d target_pos_gimbal = ballistic_solver_.transformCameraToGimbal(best_tvec);
double yaw = std::atan2(target_pos_gimbal[0], target_pos_gimbal[2]);  // in radians







/*Ballistic Model for Pitch Compensation
We implement a BallisticSolver class that calculates the required pitch angle to hit the target given the target’s distance. The model uses basic projectile motion physics to account for bullet drop due to gravity. Two modes are supported: an analytical calculation and a lookup table (LUT) mode for empirical corrections. The class also incorporates the camera-to-gimbal transform (offset) so that the input target position can be adjusted before computing pitch.

// ballistic_solver.hpp*/
#pragma once
#include <cmath>
#include <map>
#include <algorithm>
#include <opencv2/core.hpp>

namespace prm_vision {

    class BallisticSolver {
    public:
        BallisticSolver()
            : projectile_speed_(15.0), gravity_(9.81), use_analytical_(true) {
            // Default values: speed 15 m/s, gravity 9.81 m/s^2 (tune via parameters)
            // Default offsets (camera-to-gimbal) as zero; set via parameters if needed
            offset_x_ = offset_y_ = offset_z_ = 0.0;
        }

        // Configure ballistic parameters (could be called from ROS2 param update)
        void setParameters(double speed, double gravity, bool useAnalytical) {
            projectile_speed_ = speed;
            gravity_ = gravity;
            use_analytical_ = useAnalytical;
        }

        void setCameraOffset(double ox, double oy, double oz) {
            offset_x_ = ox;
            offset_y_ = oy;
            offset_z_ = oz;
        }

        // Transform 3D target position from camera frame to gimbal frame by applying the offset
        cv::Vec3d transformCameraToGimbal(const cv::Vec3d& target_cam) const {
            // Assuming camera and gimbal axes are aligned, just translate by the offset
            cv::Vec3d target_gimbal;
            target_gimbal[0] = target_cam[0] + offset_x_;
            target_gimbal[1] = target_cam[1] + offset_y_;
            target_gimbal[2] = target_cam[2] + offset_z_;
            return target_gimbal;
        }

        // Compute the required pitch angle (in radians) to hit the target at target_pos (in gimbal frame)
        double computePitchAngle(const cv::Vec3d& target_pos) const {
            // Calculate horizontal distance and vertical difference
            double X = target_pos[0];
            double Y = target_pos[1];
            double Z = target_pos[2];
            double horizontal_dist = std::sqrt(X * X + Z * Z);
            double vertical_diff = -Y;  // vertical difference (positive if target is above the camera)
            double v = projectile_speed_;
            double g = gravity_;

            if (!use_analytical_) {
                // Lookup table mode: use pre-defined mappings from distance to pitch
                // (For demonstration, a simple linear interpolation on an example table)
                static const std::map<double, double> pitch_table = {
                    {5.0,  5.0 * M_PI / 180.0},   // 5m -> 5 deg
                    {10.0, 10.0 * M_PI / 180.0},  // 10m -> 10 deg
                    {15.0, 15.0 * M_PI / 180.0}   // 15m -> 15 deg (just an example linear progression)
                };
                double dist = std::sqrt(X * X + Y * Y + Z * Z);  // straight-line distance
                auto it = pitch_table.lower_bound(dist);
                double pitch_angle = 0.0;
                if (it == pitch_table.end()) {
                    pitch_angle = pitch_table.rbegin()->second;
                }
                else if (it == pitch_table.begin()) {
                    pitch_angle = it->second;
                }
                else {
                    // Linear interpolate between it (upper bound) and previous point
                    double d2 = it->first;
                    double a2 = it->second;
                    auto it1 = std::prev(it);
                    double d1 = it1->first;
                    double a1 = it1->second;
                    double t = (dist - d1) / (d2 - d1);
                    pitch_angle = a1 + t * (a2 - a1);
                }
                return pitch_angle;
            }

            // Analytical mode: solve for the launch angle to hit target (assuming level shot for simplicity)
            // Using projectile motion formula for different elevation:
            // tanθ = (v^2 ± sqrt(v^4 - g*(g*horizontal_dist^2 + 2*vertical_diff*v^2))) / (g * horizontal_dist)
            double v2 = v * v;
            double term = v2 * v2 - g * (g * horizontal_dist * horizontal_dist + 2 * vertical_diff * v2);
            if (term < 0) {
                // No physical solution (target out of range for given speed), clamp angle to 45 degrees
                return M_PI / 4;
            }
            double sqrt_term = std::sqrt(term);
            double tan_angle1 = (v2 - sqrt_term) / (g * horizontal_dist);
            double tan_angle2 = (v2 + sqrt_term) / (g * horizontal_dist);
            double angle1 = std::atan(tan_angle1);
            double angle2 = std::atan(tan_angle2);
            // Choose the smaller positive angle (for a direct shot rather than a high-arcing shot)
            double pitch_angle = angle1;
            if (angle2 > 0 && angle2 < angle1) {
                pitch_angle = angle2;
            }
            return pitch_angle;
        }

    private:
        double projectile_speed_;  // projectile muzzle speed (m/s)
        double gravity_;           // gravity (m/s^2)
        bool use_analytical_;      // true for analytical model, false for LUT mode

        // Camera-to-gimbal offset (in meters, in camera frame axes)
        double offset_x_, offset_y_, offset_z_;
    };

} // namespace prm_vision
