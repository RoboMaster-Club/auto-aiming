#include <vector>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>

#define PI 3.141592653589793238462643383

// max robot radius in mm
#define MAX_ROBOT_RADIUS 250

// initial robot radius estimate in mm
#define INITIAL_RADIUS_ESTIMATE 230 

// bullet speed in mm/ms
#define BULLET_SPEED 30

// the rotation absolute value of an armor to giveup tracking
#define GIVEUP_ROTATION_ABS 40

// bounds for detecting outliers of yaw change
#define BOXCAR_AVG_MULTIPLE_BOUND 2
#define BOXCAR_AVG_CONSTANT_BOUND 0.2


typedef struct Coordinates {
    cv::Vec3d tvec;
    cv::Vec3d rvec;
} Coordinates;

class AntiSpintop {
    public:
        AntiSpintop();
        AntiSpintop(size_t buffer_size, double slope_threshold);
        
        // getters for debugging purposes
        double get_avg();
        double get_radius();
        double get_yaw();
        int get_direction();

        void calculate_aim_point(cv::Mat* tvecs, double* yaws, int armor_found, cv::Mat& aim_tvec, double& aim_yaw, int delta_time = -1);
        
    private:
        int delta_time;
        int spin_direction;
        int rvec_indexes[2];
        bool armor_detected[2];
        double target_radius;
        double prev_rotation;
        double boxcar_average;
        double slope_threshold;

        double current_yaw;

        size_t buffer_size;
        std::vector<double> yaw_buffer;
        std::chrono::steady_clock::time_point last_update_time;

        double future_approx(cv::Mat tvec, double rotation);
        double estimate_radius(cv::Mat* tvecs, double* yaws, int armor_found);

        int check_spinning(double yaw, double threshold);
};
