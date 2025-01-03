#include "ControlCommunicator.hpp"

ControlCommunicator::ControlCommunicator() {
}

ControlCommunicator::ControlCommunicator(const char *port) {
    this->set_port(port);
}

ControlCommunicator::~ControlCommunicator()
{
    close(this->port_fd);
}

bool ControlCommunicator::connected() {
    return this->is_connected;
}
int ControlCommunicator::get_port_fd() {
    return this->port_fd;
}

void ControlCommunicator::set_port(const char *port) {
    this->port = port;
    this->start_uart();
}

/**
 * @brief Starts the UART interface with the STM32 board
 * 
 * @return Error message for starting UART (Empty if successful)
 */
std::string ControlCommunicator::start_uart() {
    this->is_connected = false;
    this->port_fd = open(this->port, O_RDWR);

    // Check for errors
    if (this->port_fd < 0)
    {
        std::stringstream ss;
        ss << "Failed to open: " << this->port << ", " << strerror(errno);
        return ss.str();
    }

    struct termios tty;

    // Set UART TTY to 8n1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;	   // No RTS/CTS flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
    tty.c_lflag &= ~ICANON;		   // Disable canonical mode

    // Disable echo, erasure and newline echo
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;

    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~ISIG;

    // Disable special handling, interpretation, S/W flow control, \n conv.
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;				// Wait for up to 1s (10 deciseconds)
    tty.c_cc[VMIN] = sizeof(PackageIn); // Block for sizeof(PackageOut) bits

    // Set the baud rate
    cfsetispeed(&tty, B1152000);

    // Save tty settings, also checking for error
    if (tcsetattr(this->port_fd, TCSANOW, &tty) != 0)
    {
        std::stringstream ss;
        ss << "Error " << std::to_string(errno) << " from tcsetattr: " << strerror(errno);
        return ss.str();
    }
    this->is_connected = true;

    return std::string();
}

/**
 * @brief Sends a heart beat packet to the STM32 board
 * 
 * @return whether the write operation is successful
 */
int ControlCommunicator::send_heart_beat_packet() {
    PackageOut package;
    package.frame_id = 0xAA;
    package.frame_type = FRAME_TYPE_HEART_BEAT;
    package.heartBeatPackage._a = 0xAA;
    package.heartBeatPackage._b = 0xAA;
    package.heartBeatPackage._c = 0xAA;
    package.heartBeatPackage._d = 0xAA;
    int success = write(this->port_fd, &package, sizeof(PackageOut));
    fsync(this->port_fd);
    
    if (success == -1) {
        this->is_connected = false;
    }

    return success;
}

/**
 * @brief Sends an auto aim packet to the STM32 board
 * 
 * @return whether the write operation is successful
 */
int ControlCommunicator::send_auto_aim_packet(float yaw, float pitch, bool fire) {
    PackageOut package;
    package.frame_id = 0xAA;
    package.frame_type = FRAME_TYPE_AUTO_AIM;
    package.autoAimPackage.yaw = yaw;
    package.autoAimPackage.pitch = pitch;
    package.autoAimPackage.fire = fire;
    int success = write(this->port_fd, &package, sizeof(PackageOut));
    fsync(this->port_fd);

    return success;
}

/**
 * @brief Sends a nav packet to the STM32 board
 * 
 * @return whether the write operation is successful
 */
int ControlCommunicator::send_nav_packet(float x_vel, float y_vel, float yaw_rad, uint8_t state) {
    PackageOut package;
    package.frame_id = 0xAA;
    package.frame_type = FRAME_TYPE_NAV;
    package.navPackage.x_vel = x_vel;
    package.navPackage.y_vel = y_vel;
    package.navPackage.yaw_rad = yaw_rad;
    package.navPackage.state = state;
    int success = write(this->port_fd, &package, sizeof(PackageOut));
    fsync(this->port_fd);

    return success;
}

/**
 * @brief Attempts to align packet reading
 * 
 * @return true if the packet is successfully aligned
 * @return false if the packet fails to align
 */
bool ControlCommunicator::read_alignment() {
    uint8_t i = 0;
    uint8_t buffer[32];
    do
    {
        int success = read(this->port_fd, &(buffer[0]), sizeof(buffer[0]));
        if (success) i++;
    } while (buffer[0] != 0xAA || i > sizeof(PackageIn) * 2);
    read(this->port_fd, &buffer, sizeof(PackageIn) - 1);

    return i <= sizeof(PackageIn) * 2;
}

/**
 * @brief Reads in a packet from the STM32 board
 * 
 * @return A tuple containing the error message (empty if successful) and the
 * package struct read in
 */
std::tuple<std::string, PackageIn> ControlCommunicator::read_package() {
    PackageIn package;
    int success = read(this->port_fd, &package, sizeof(PackageIn));

    if (success == -1) {
        std::stringstream ss;
        ss << "Error " << errno << " from read: " << strerror(errno);
        return std::tuple<std::string, PackageIn>(ss.str(), package);
    }

    // Package validation
    if (package.head != 0xAA) {
        return this->read_alignment()
            ? std::tuple<std::string, PackageIn>("Read alignment success.", package)
            : std::tuple<std::string, PackageIn>("Read alignment failed.", package);
    }

    return std::tuple<std::string, PackageIn>(std::string(), package);
}