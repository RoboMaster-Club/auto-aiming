#include <gtest/gtest.h>
#include "ControlCommunicator.h"

class ControlCommunicatorTest : public ::testing::Test
{
protected:
    const char *port = nullptr;

    void SetUp() override
    {
        // Find which port to use
        const char *port1 = "/dev/ttyUSB0";
        const char *port2 = "/dev/ttyUSB1";
        bool port1_exists = access(port1, F_OK) != -1;
        bool port2_exists = access(port2, F_OK) != -1;

        // If either of the ports exist, use it
        if (port1_exists || port2_exists)
        {
            if (port1_exists)
            {
                port = port1;
            }
            else
            {
                port = port2;
            }
        }
        else
        {
            // Neither port exists on dev server or local machine, since no serial device
            port = nullptr;
        }
    }

    void TearDown() override
    {
    }
};

TEST_F(ControlCommunicatorTest, test_start_uart_connection)
{
    ControlCommunicator control_communicator;
    if (port != nullptr)
    {
        EXPECT_TRUE(control_communicator.start_uart_connection(port));

        /**
         * Check UART configuration
         */
        struct termios tty;
        int port_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(port_fd, -1); // Port should be opened

        if (tcgetattr(port_fd, &tty) != 0)
        {
            close(port_fd);
            FAIL();
        }

        // Check baud rate
        EXPECT_EQ(cfgetispeed(&tty), B1152000);
        EXPECT_EQ(cfgetospeed(&tty), B1152000);

        // Check parity, stop bits, data bits, flow control, read and local mode
        EXPECT_EQ(tty.c_cflag & PARENB, 0);
        EXPECT_EQ(tty.c_cflag & CSTOPB, 0);
        EXPECT_EQ(tty.c_cflag & CSIZE, 0);
        EXPECT_EQ(tty.c_cflag & CS8, CS8);
        EXPECT_EQ(tty.c_cflag & CRTSCTS, 0);
        EXPECT_EQ(tty.c_cflag & CREAD, CREAD);
        EXPECT_EQ(tty.c_cflag & CLOCAL, CLOCAL);

        // Check VMIN and VTIME
        EXPECT_EQ(tty.c_cc[VMIN], 1);
        EXPECT_EQ(tty.c_cc[VTIME], 10);
    }
    else
    {
        // Neither port exists on dev server or local machine, since no serial device
        EXPECT_FALSE(control_communicator.start_uart_connection(port));
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}