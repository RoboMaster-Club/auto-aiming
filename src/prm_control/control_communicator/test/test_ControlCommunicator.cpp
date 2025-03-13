#include <gtest/gtest.h>
#include "ControlCommunicator.h"
#include "ControlCommunicatorNode.hpp"

class ControlCommunicatorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
};

TEST_F(ControlCommunicatorTest, test_start_uart_connection)
{
    ControlCommunicator control_communicator;
    const char *port1 = "/dev/ttyUSB0";
    const char *port2 = "/dev/ttyUSB1";

    // See if either of the ports exist on file system
    bool port1_exists = access(port1, F_OK) != -1;
    bool port2_exists = access(port2, F_OK) != -1;

    // If either of the ports exist, try to connect to it
    if (port1_exists)
    {
        EXPECT_TRUE(control_communicator.start_uart_connection(port1));
    }
    else if (port2_exists)
    {
        EXPECT_TRUE(control_communicator.start_uart_connection(port2));
    }
    else
    {
        // Neither port exists on dev server or local machine, since no serial device
        EXPECT_FALSE(control_communicator.start_uart_connection(port1));
        EXPECT_FALSE(control_communicator.start_uart_connection(port2));
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}