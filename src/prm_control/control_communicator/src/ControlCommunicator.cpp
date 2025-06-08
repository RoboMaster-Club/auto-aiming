#include "ControlCommunicator.h"
#include "projectile_angle_convel.h"

bool ControlCommunicator::start_uart_connection(const char *port)
{
    this->port = port;
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

void ControlCommunicator::load_robot_lookup_table()
{
   // Gets device hostname and loads "hostname_lookup_table.txt" file. This is the standard naming for lookup tables. If
   // hostname is not found, loads "pitch_lookup_table.txt", which should be the swerve standard lookup table at time of writing.
   
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, sizeof(hostname));
    std::string hostname_str(hostname);

    // TODO:: replace with actual host names
    
    std::string lut_path;

    if (hostname_str.compare("swerve_standard") == 0)
    {
        lut_path = "/home/purduerm/ros2-ws/auto-aiming/src/prm_control/control_communicator/include/lookup_tables/pitch_lookup_table.txt";
    }
    else if (hostname_str.compare("hero") == 0)
    {
        lut_path = "/home/purduerm/ros2-ws/auto-aiming/src/prm_control/control_communicator/include/lookup_tables/pitch_lookup_table.txt";
    }
    else if (hostname_str.compare("sentry") == 0)
    {
        lut_path = "/home/purduerm/ros2-ws/auto-aiming/src/prm_control/control_communicator/include/lookup_tables/pitch_lookup_table.txt";
    }
    else
    {
        lut_path = "/home/purduerm/ros2-ws/auto-aiming/src/prm_control/control_communicator/include/lookup_tables/pitch_lookup_table.txt";
    }
    this->lut = new PitchLookupModel(lut_path);
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

    // projectile model based on quartic solver
    bool impossible = false;
    double p;
    double y;
    pitch_yaw_gravity_model_movingtarget_const_v({ target_z, target_x, -target_y }, {0, 0, 0}, {0, 0, 9810}, 0.0, &p, &y, &impossible);
 
    pitch = -(float)p;
    yaw = (float)y * -(target_x / abs(target_x)); // yaw is always returned positive, so multiply by sign of target_x

    // lookup table for empirical pitch correction
    float dst = sqrt(target_x * target_x + target_y * target_y + target_z * target_z);
    pitch += lut->get_offset(dst, target_y);

    // check nan and ensure between -180 and 180
    if (std::isnan(pitch) || std::isnan(yaw) ||
        pitch > 180 || pitch < -180 ||
        yaw > 180 || yaw < -180)
    {
        pitch = 0;
        yaw = 0;
    }
}

bool ControlCommunicator::read_uart(int port_fd, PackageIn &package, const char *port)
{
    // Node's read_uart is on a timer, so it will be called every 4ms
    // Prevents reading if we are already in the process of reconnecting
    static bool reconnecting = false;
    if (reconnecting)
    {
        return false;
    }

    int success = read(port_fd, &package, sizeof(PackageIn));
    if (success == -1)
    {
        return false; // No data to read or error
    }

    if (package.head != 0xAA)
    {
        reconnecting = true;
        tcflush(port_fd, TCIOFLUSH);
        close(port_fd);

        // Reconnecting
        this->start_uart_connection(port);

        reconnecting = false;
        return false;
    }

    return true;
}
