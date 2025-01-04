#include <gtest/gtest.h>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "OpenCVArmorDetector.h"
#include "PNPSolver.h"


#include <fstream>
#include <string>
#include <vector>
#include <sstream>

// Helpers
void testVideo(std::string video_path, OpenCVArmorDetector *detector, float min_detection_rate, float max_loss_pix);

/* DETECTOR REQUIREMENTS */
// Performance requirements
#define MIN_FPS 80 // Minimum FPS required

// Pre-selected images (tighter requirements since we know an armor is present)
#define MIN_DETECTION_RATE_EASY 0.95     // 95% detection rate required
#define MIN_DETECTION_RATE_FAR_BACK 0.90 // 90% detection rate required
#define MAX_LOSS_EASY 2                  // 2 pixels loss allowed
#define MAX_LOSS_FAR_BACK 3              // 3 pixels loss allowed

// Realistic videos (looser requirements due to spintop and obstruction)
#define MIN_DETECTION_RATE_FAR_BACK_SPIN_AND_MOVE 0.70 // 70% detection rate required
#define MIN_DETECTION_RATE_CLOSE_VIDEO 0.65            // 65% detection rate required
#define MAX_LOSS_FAR_BACK_SPIN_AND_MOVE 3              // 3 pixels loss allowed
#define MAX_LOSS_CLOSE_VIDEO 3                         // 3 pixels loss allowed
/* END DETECTOR REQUIREMENTS */

class PNPSolverTest : public ::testing::Test
{
protected:
    OpenCVArmorDetector *detector;
    PNPSolver * pnp_solver

    void SetUp() override
    {
        DetectorConfig config = {RED, 30, 100, 150, 1, true};
        detector = new OpenCVArmorDetector(config);
        pnp_solver = new PNPSolver();
    }

    void TearDown() override
    {
        delete detector;
    }
};


TEST_F(PNPSolverTest, spinning_in_place)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    OpenCVArmorDetector *detector_close = new OpenCVArmorDetector({RED, 30, 100, 150, 1, true});

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector_close->getMissedFrames(), 0);
    EXPECT_EQ(detector_close->_detected_frame, 0);
    EXPECT_EQ(detector_close->_frame_count, 0);

    std::string videoPath = "/home/user-accounts/public/spintop/spinning_in_place.avi";
    testVideo(videoPath, detector_close, MIN_DETECTION_RATE_CLOSE_VIDEO, MAX_LOSS_CLOSE_VIDEO);
}

void testVideo(std::string video_path, OpenCVArmorDetector *detector, float min_detection_rate, float max_loss_pix)
{
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened())
    {
        std::cerr << "Error opening video file: " << video_path << std::endl;
        FAIL();
    }

    cv::Mat frame;

    float total_loss = 0;
    int frame_idx = 0;

    FILE* fp = fopen("gt.csv", "w");
    while (cap.read(frame))
    {
        cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));
        std::vector<_Float32> points = detector->search(frame);
        std::vector<float> point_array;
        std::copy(points.begin(), points.end(), points_array.begin());

        Coordinates coordinates = PNPSolver::getArmorCoordinates(point_array, point_array.size(), false);
        fprintf(fp, "%.5f,%.5f,%.5f\n", coordinates.tvec);
    }
    fclose(fp);
    EXPECT_EQ(0, 0);
}
