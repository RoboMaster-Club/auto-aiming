#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <Eigen/Dense>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include <errno.h> // Error integer and strerror() function

#include "utils.hpp"
#include "messages.hpp"
#include "time_debug.hpp"

class ControlCommunicator
{
public:
    ControlCommunicator();
    ControlCommunicator(const char *port);
    ~ControlCommunicator();

    bool connected();
    int get_port_fd();
    void set_port(const char *port);
    std::string start_uart();
    int send_heart_beat_packet();
    int send_auto_aim_packet(float yaw, float pitch, bool fire);
    int send_nav_packet(float x_vel, float y_vel, float yaw_rad, uint8_t state);
    bool read_alignment();
    std::tuple<std::string, PackageIn> read_package();
private:
    const char *port;
    int port_fd = -1;
    bool is_connected = false;
};