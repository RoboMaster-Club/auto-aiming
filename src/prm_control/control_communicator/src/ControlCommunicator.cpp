#include "ControlCommunicator.h"
#include "projectile_angle_convel.h"

bool ControlCommunicator::start_uart_connection(const char *port)
{
    this->is_connected = false;
    this->port_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    tcflush(this->port_fd, TCIOFLUSH);

    if (this->port_fd == -1)
    {
        return false; // Error opening the port
    }

    struct termios tty;
    if (tcgetattr(this->port_fd, &tty) != 0)
    {
        close(this->port_fd);
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

    if (tcsetattr(this->port_fd, TCSANOW, &tty) != 0)
    {
        close(this->port_fd);
        return false;
    }

    is_connected = true;
    return true;
}

void ControlCommunicator::compute_aim(float bullet_speed, float target_x, float target_y, float target_z, float &yaw, float &pitch)
{
    // if X and Y and Z are 0
    if (target_x == 0 && target_y == 0 && target_z == 0)
    {
        yaw = 0;
        pitch = 0;
        return;
    }
    else
    {
        float dst_horiz = sqrt(target_x * target_x + target_z * target_z);
        yaw = -atan(target_x / target_z) * 180 / M_PI;
        pitch = atan(target_y / dst_horiz) * 180 / M_PI;
    }

    // projectile angle convel
    vec3 pos = { target_z, target_x, -target_y };
    vec3 vel = {0, 0, 0};
    vec3 g = {0, 0, 9810};
    bool impossible = false;
    double p;
    double y;
    pitch_yaw_gravity_model_movingtarget_const_v(pos, vel, g, 0.0, &p, &y, &impossible);

    pitch = -(float)p;
    yaw = (float)y * -(target_x / abs(target_x));

    // Lookup Table
    // float dst = sqrt(target_x * target_x + target_y * target_y + target_z * target_z);
    // float flt = lut->get_pitch(dst, -target_y);
    // pitch = pitch + flt;
}
