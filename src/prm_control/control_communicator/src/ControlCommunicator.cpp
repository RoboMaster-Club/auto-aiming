#include "ControlCommunicator.h"
#include "projectile_angle_convel.h"

bool ControlCommunicator::start_uart_connection(const char *port)
{
	this->is_connected = false;
	this->port_fd = open(port, O_RDWR);

	// Check for errors
	if (this->port_fd < 0)
	{
		return false;
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
		return false;
	}
	this->is_connected = true;

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