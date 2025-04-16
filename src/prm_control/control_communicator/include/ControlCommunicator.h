#ifndef CONTROL_COMMUNICATOR_HPP
#define CONTROL_COMMUNICATOR_HPP

#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include "PitchLookupModel.hpp"

class ControlCommunicator
{
public:
    ControlCommunicator() {}
    ~ControlCommunicator() {}

    // Class methods
    void compute_aim(float bullet_speed, float target_x, float target_y, float target_z, float &yaw, float &pitch);
    bool start_uart_connection(const char *port);
    void read_uart();

    PitchLookupModel *lut = new PitchLookupModel("lookup_tables/pitch_lookup_table.txt");

    int port_fd = -1;
    bool is_connected = false;
private:
};

#endif // CONTROL_COMMUNICATOR_HPP