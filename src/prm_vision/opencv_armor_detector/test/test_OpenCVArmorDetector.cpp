#include <gtest/gtest.h>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "OpenCVArmorDetector.h"

// Helper
void iterateThroughFolder(std::string folder_path, OpenCVArmorDetector *detector);

// DETECTOR REQUIREMENTS
#define MIN_EASY_ACCURACY 0.95 // 95% accuracy
#define MAX_EASY_LOSS_PIX 5    // 5 pixels loss allowed

// Set up the test fixture
class OpenCVArmorDetectorTest : public ::testing::Test
{
protected:
    OpenCVArmorDetector *detector;

    void SetUp() override
    {
        DetectorConfig config = {RED, 30, 100, 150, 2, true};
        detector = new OpenCVArmorDetector(config);
    }

    void TearDown() override
    {
        delete detector;
    }
};

////////////////////////
// START CONFIG TESTS //
////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_Config)
{
    DetectorConfig c = detector->getConfig();
    EXPECT_EQ(c._target_color, RED);
    EXPECT_EQ(c._hue_range_limit, 30);
    EXPECT_EQ(c._saturation_lower_limit, 100);
    EXPECT_EQ(c._value_lower_limit, 150);
    EXPECT_EQ(c._max_missed_frames, 2);
    EXPECT_EQ(c._reduce_search_area, true);
}

TEST_F(OpenCVArmorDetectorTest, test_SetConfig)
{
    std::cout << "Testing setConfig" << std::endl;
    DetectorConfig config = {BLUE, 1, 2, 3, 4, false};
    detector->setConfig(config);
    DetectorConfig c = detector->getConfig();
    EXPECT_EQ(c._target_color, BLUE);
    EXPECT_EQ(c._hue_range_limit, 1);
    EXPECT_EQ(c._saturation_lower_limit, 2);
    EXPECT_EQ(c._value_lower_limit, 3);
    EXPECT_EQ(c._max_missed_frames, 4);
    EXPECT_EQ(c._reduce_search_area, false);
    detector->setConfig({RED, 30, 100, 150, 2, true}); // Reset the config
}
//////////////////////
// END CONFIG TESTS //
//////////////////////

////////////////////////////////////////
// START isLightBar AND isArmor TESTS //
//    We want 100% branch coverage    //
////////////////////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_isLightBarFalses)
{
    // Width too small (LIGHT_BAR_WIDTH_LOWER_LIMIT)
    cv::RotatedRect rect1(cv::Point2f(0, 0), cv::Size2f(LIGHT_BAR_WIDTH_LOWER_LIMIT - 0.1, 100), 0);
    EXPECT_FALSE(detector->isLightBar(rect1));

    // Height too small (LIGHT_BAR_HEIGHT_LOWER_LIMIT)
    cv::RotatedRect rect2(cv::Point2f(0, 0), cv::Size2f(100, LIGHT_BAR_HEIGHT_LOWER_LIMIT - 0.1), 0);
    EXPECT_FALSE(detector->isLightBar(rect2));

    // Roll angle more than LIGHT_BAR_ANGLE_LIMIT from the horizontal
    cv::RotatedRect rect3(cv::Point2f(0, 0), cv::Size2f(100, 100), LIGHT_BAR_ANGLE_LIMIT + 1);
    cv::RotatedRect rect4(cv::Point2f(0, 0), cv::Size2f(100, 100), 180 - LIGHT_BAR_ANGLE_LIMIT - 1);
    EXPECT_FALSE(detector->isLightBar(rect3));
    EXPECT_FALSE(detector->isLightBar(rect4));

    // Aspect ratio (h/w) too low (LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT)
    cv::RotatedRect rect5(cv::Point2f(0, 0), cv::Size2f(100, (LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT * 100) - 0.1), 0);
    EXPECT_FALSE(detector->isLightBar(rect5));

    // MIX ALL THE ABOVE FALSES
    // Angle out of bounds and width too small
    cv::RotatedRect rect6(cv::Point2f(0, 0), cv::Size2f(LIGHT_BAR_WIDTH_LOWER_LIMIT - 0.1, 100), 90);
    EXPECT_FALSE(detector->isLightBar(rect6));

    // Angle out of bounds and height too small
    cv::RotatedRect rect7(cv::Point2f(0, 0), cv::Size2f(100, LIGHT_BAR_HEIGHT_LOWER_LIMIT - 0.1), 150);
    EXPECT_FALSE(detector->isLightBar(rect7));

    // Width too small and height too small
    cv::RotatedRect rect8(cv::Point2f(0, 0), cv::Size2f(LIGHT_BAR_WIDTH_LOWER_LIMIT - 0.1, LIGHT_BAR_HEIGHT_LOWER_LIMIT - 0.1), 0);
    EXPECT_FALSE(detector->isLightBar(rect8));
}

TEST_F(OpenCVArmorDetectorTest, test_isLightBarTrue)
{
    // Check some light bars which should pass (not using defined limits)
    cv::RotatedRect rect1(cv::Point2f(0, 0), cv::Size2f(40, 135), 0);
    cv::RotatedRect rect2(cv::Point2f(0, 0), cv::Size2f(40, 135), 15);
    cv::RotatedRect rect3(cv::Point2f(0, 0), cv::Size2f(40, 135), 180 - 15);
    cv::RotatedRect rect4(cv::Point2f(0, 0), cv::Size2f(40, 135), 180);
    cv::RotatedRect rect5(cv::Point2f(0, 0), cv::Size2f(40, 135), 180 + 15);

    // Generate 500 random light bars meeting the criteria
    for (int i = 0; i < 500; i++)
    {
        // width >= LIGHT_BAR_WIDTH_LOWER_LIMIT
        double width = LIGHT_BAR_WIDTH_LOWER_LIMIT + (rand() % 100);

        // height >= LIGHT_BAR_HEIGHT_LOWER_LIMIT, and height >= width * LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT
        // So height is whichever is bigger between LIGHT_BAR_HEIGHT_LOWER_LIMIT and width * LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT (plus a bit of randomness)
        double height = std::max(LIGHT_BAR_HEIGHT_LOWER_LIMIT, width * LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT) + static_cast<double>(rand() % 100);

        // angle in [0, LIGHT_BAR_ANGLE_LIMIT] or [180 - LIGHT_BAR_ANGLE_LIMIT, 180]
        double angle = i % 2 == 0 ? rand() % (int)LIGHT_BAR_ANGLE_LIMIT : 180 - (rand() % (int)LIGHT_BAR_ANGLE_LIMIT);

        cv::RotatedRect rect(cv::Point2f(0, 0), cv::Size2f(width, height), angle);
        EXPECT_TRUE(detector->isLightBar(rect));
    }
}

TEST_F(OpenCVArmorDetectorTest, test_isArmorFalses)
{
    // Angle difference too large (ARMOR_ANGLE_DIFF_LIMIT)
    cv::RotatedRect left_rect1(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect1(cv::Point2f(0, 0), cv::Size2f(100, 100), ARMOR_ANGLE_DIFF_LIMIT + 1);
    EXPECT_FALSE(detector->isArmor(left_rect1, right_rect1));

    // Aspect ratio ratio difference too large (ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT)
    cv::RotatedRect left_rect2(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect2(cv::Point2f(0, 0), cv::Size2f(100, 600), 0);
    EXPECT_FALSE(detector->isArmor(left_rect2, right_rect2));

    // Distance too small or too large
    cv::RotatedRect left_rect3(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect3(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    EXPECT_FALSE(detector->isArmor(left_rect3, right_rect3));

    // X position distance too small
    cv::RotatedRect left_rect4(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect4(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    EXPECT_FALSE(detector->isArmor(left_rect4, right_rect4));

    // Light bar Y position ratio too large
    cv::RotatedRect left_rect5(cv::Point2f(0, 0), cv::Size2f(100, 100), 0);
    cv::RotatedRect right_rect5(cv::Point2f(0, 60), cv::Size2f(100, 100), 0);
    EXPECT_FALSE(detector->isArmor(left_rect5, right_rect5));
}

///////////////////////////////////////////////////
//       END isLightBar AND isArmor TESTS        //
// We will test the true case in the search test //
///////////////////////////////////////////////////

////////////////////////
// START search TESTS //
////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_search_no_armor)
{
    // Test the search method with a blank frame
    EXPECT_EQ(detector->getMissedFrames(), 0);
    EXPECT_EQ(detector->_detected_frame, 0);

    cv::Mat frame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    std::vector<_Float32> points = detector->search(frame);
    EXPECT_EQ(points.size(), 8);
    for (int i = 0; i < 8; i++)
    {
        EXPECT_EQ(points.at(i), 0);
    }

    EXPECT_EQ(detector->getMissedFrames(), 1);
    EXPECT_EQ(detector->_detected_frame, 0);
}

//
// This is the key test section for the armor detection. We will test the search method with frames containing armor
// We must meet an accuracy and loss threshold to pass
//
TEST_F(OpenCVArmorDetectorTest, test_search_armor_easy)
{
    // We install the test resources to the package share directory (done in the CMakeLists.txt)
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("opencv_armor_detector");

    // Should be no missed frames or detected frames at the start
    EXPECT_EQ(detector->getMissedFrames(), 0);
    EXPECT_EQ(detector->_detected_frame, 0);
    EXPECT_EQ(detector->_frame_count, 0);

    // TEST THE EASY CASES
    OpenCVArmorDetector *detector_easy = new OpenCVArmorDetector({RED, 30, 100, 150, 2, false});
    std::filesystem::path easy_path = std::filesystem::path(package_share_dir) / "resources/easy";
    iterateThroughFolder(easy_path.string(), detector_easy);
    EXPECT_EQ(detector_easy->_frame_count, 158); // 159 frames in the easy folder
    double detection_accuracy = static_cast<double>(detector_easy->_detected_frame) / static_cast<double>(detector_easy->_frame_count);
    EXPECT_GE(detection_accuracy, MIN_EASY_ACCURACY);

    // TODO: Check loss against ground truths

    delete detector_easy;
}

void iterateThroughFolder(std::string folder_path, OpenCVArmorDetector *detector)
{
    namespace fs = std::filesystem;

    // Iterate through all .png files in the easy tests folder
    for (const auto &entry : fs::directory_iterator(folder_path))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".png")
        {
            std::string file_path = entry.path().string();

            // Load the image
            cv::Mat frame = cv::imread(file_path, cv::IMREAD_COLOR);
            if (frame.empty())
            {
                std::cerr << "Failed to read image: " << file_path << std::endl;
                continue;
            }

            cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));

            // Call the detector's search function
            std::vector<_Float32> points = detector->search(frame);
        }
    }
}
