#include <gtest/gtest.h>
#include "ControlCommunicator.h"  // Include the header file for ControlCommunicator

class ControlCommunicatorTest : public ::testing::Test {
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}