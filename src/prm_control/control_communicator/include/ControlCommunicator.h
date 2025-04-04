#ifndef CONTROL_COMMUNICATOR_HPP
#define CONTROL_COMMUNICATOR_HPP

#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

class ControlCommunicator
{
public:
    ControlCommunicator() {}
    ~ControlCommunicator() {}

    // Class methods
    void compute_aim(float bullet_speed, float target_x, float target_y, float target_z, float &yaw, float &pitch);
    bool start_uart_connection(const char *port);
    void read_uart();

    int port_fd = -1;
private:
};

#endif // CONTROL_COMMUNICATOR_HPP