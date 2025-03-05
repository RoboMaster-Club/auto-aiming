#ifndef CONTROL_COMMUNICATOR_HPP
#define CONTROL_COMMUNICATOR_HPP

class ControlCommunicator
{
public:
    ControlCommunicator() {}
    ~ControlCommunicator();

    // Class methods
    void compute_aim(float bullet_speed, float target_x, float target_y, float target_z, float &yaw, float &pitch);
    bool start_uart_connection(const char *port);
    void read_uart();

private:
};

#endif // CONTROL_COMMUNICATOR_HPP