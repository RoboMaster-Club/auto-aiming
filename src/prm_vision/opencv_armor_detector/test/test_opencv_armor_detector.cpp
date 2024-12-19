#include <gtest/gtest.h>
#include "OpenCVArmorDetector.h"

// Set up the test fixture
class OpenCVArmorDetectorTest : public ::testing::Test
{
protected:
    OpenCVArmorDetector *detector;

    void SetUp() override
    {
        DetectorConfig config = {RED, 30, 100, 150, 2};
        detector = new OpenCVArmorDetector(config);
    }

    void TearDown() override
    {
        delete detector;
    }
};

/////////////////////////
/// START CONFIG TESTS //
/////////////////////////
TEST_F(OpenCVArmorDetectorTest, test_Config)
{
    DetectorConfig c = detector->getConfig();
    EXPECT_EQ(c._target_color, RED);
    EXPECT_EQ(c._hue_range_limit, 30);
    EXPECT_EQ(c._saturation_lower_limit, 100);
    EXPECT_EQ(c._value_lower_limit, 150);
    EXPECT_EQ(c._max_missed_frames, 2);
}

TEST_F(OpenCVArmorDetectorTest, test_SetConfig)
{
    std::cout << "Testing setConfig" << std::endl;
    DetectorConfig config = {BLUE, 1, 2, 3, 4};
    detector->setConfig(config);
    DetectorConfig c = detector->getConfig();
    EXPECT_EQ(c._target_color, BLUE);
    EXPECT_EQ(c._hue_range_limit, 1);
    EXPECT_EQ(c._saturation_lower_limit, 2);
    EXPECT_EQ(c._value_lower_limit, 3);
    EXPECT_EQ(c._max_missed_frames, 4);
    detector->setConfig({RED, 30, 100, 150, 2}); // Reset the config
}
