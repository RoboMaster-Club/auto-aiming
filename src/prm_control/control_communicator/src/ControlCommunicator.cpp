#include "ControlCommunicator.h"
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

bool ControlCommunicator::start_uart_connection(const char *port)
{
    int port_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (port_fd == -1)
    {
        return false; // Error opening the port
    }

    struct termios tty;
    if (tcgetattr(port_fd, &tty) != 0)
    {
        close(port_fd);
        return false;
    }

    int baud_rate = B1152000;
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable read and local mode

    cfmakeraw(&tty);      // Raw mode
    tty.c_cc[VMIN] = 1;   // Minimum 1 character
    tty.c_cc[VTIME] = 10; // Timeout in deciseconds (1 second)

    if (tcsetattr(port_fd, TCSANOW, &tty) != 0)
    {
        close(port_fd);
        return false;
    }

    return true;
}

void ControlCommunicator::compute_aim(float bullet_speed, float target_x, float target_y, float target_z, float &yaw, float &pitch)
{
    float g = 9.8f; // Gravity
    float v = bullet_speed;
    float dx = target_x;
    float dy = target_y;
    float dz = target_z;
    float d = sqrt(dx * dx + dy * dy + dz * dz);
    float t = d / v;
    float vx = dx / t;
    float vy = dy / t;
    float vz = dz / t - 0.5f * g * t;
    yaw = atan2(vy, vx);
    pitch = atan2(vz, sqrt(vx * vx + vy * vy));
}
