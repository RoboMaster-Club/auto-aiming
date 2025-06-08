#ifndef CONTROL_COMMUNICATOR_HPP
#define CONTROL_COMMUNICATOR_HPP

#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include "PitchLookupModel.hpp"
#include "messages.h"

class ControlCommunicator
{
public:
    ControlCommunicator() {}
    ~ControlCommunicator() {}

    // Class methods
    void compute_aim(float bullet_speed, float target_x, float target_y, float target_z, float &yaw, float &pitch);
    bool start_uart_connection(const char *port);
    bool read_uart(int port_fd, PackageIn &package, const char *port);
    void load_robot_lookup_table();

    PitchLookupModel *lut;

    int port_fd = -1;
    const char *port;

    bool is_connected = false;
private:
};

#endif // CONTROL_COMMUNICATOR_HPP