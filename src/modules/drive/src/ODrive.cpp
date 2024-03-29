#include "ODrive.hpp"

/*
ODrive starts by opening UART connection
*/
ODrive::ODrive(const std::string port, const rclcpp::Node *node) : node(node)
{
    // this.node = node;
    fd = open_port(port);
    if (fd == -1)
    {
        RCLCPP_INFO(node->get_logger(), "ODrive could not open UART port\n");
        throw std::string("ODrive could not open UART port");
    }
}

/*
Cleanup by closing UART connection
*/
ODrive::~ODrive()
{
    flock(fd, LOCK_UN);
    close(fd);
    RCLCPP_INFO(node->get_logger(), "Odrive closed");
}


/*
returns file descriptor with baudrate 115200,
returns -1 if oening fails
*/
int ODrive::open_port(const std::string port = "/dev/ttyS1")
{
    // file descriptor
    int fd;
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        close(fd);
        RCLCPP_WARN(node->get_logger(), "open_port: Unable to open %s", port.c_str());
        return -1;
    }

    // lock to prevent others from interfering with the node
    if (flock(fd, LOCK_EX | LOCK_NB) == -1)
    {
        flock(fd, LOCK_UN);
        close(fd);
        RCLCPP_WARN(node->get_logger(), "Serial port with file descriptor %s is already locked by another process.", std::to_string(fd));
        return -1;
    }

    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
    {
        flock(fd, LOCK_UN);
        close(fd);
        RCLCPP_WARN(node->get_logger(), "error from tggetattr");
        return -1;
    }

    // disable parity
    tty.c_cflag &= ~PARENB;

    // set one stop bit
    tty.c_cflag &= ~CSTOPB;

    // Set baud rate | 8 bits per byte | ignore ctrl lines | Turn on READ (no modem signals)
    tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

    // flow control(signal readyness)
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~ICANON;

    // disble echoing of signal
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo

    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~ISIG;

    // disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Disabling Special Handling Of Bytes On Receive
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Wait for up to .1s (1 deciseconds), return when message is finished
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        flock(fd, LOCK_UN);
        close(fd);
        RCLCPP_INFO(node->get_logger(), "Error %i from tcsetattr: ", strerror(errno));
        return -1;
    }

    return (fd);
}

/*
bitwise and each char
*/
int ODrive::checksum(const std::string cmd)
{
    int cs = 0;
    for (long unsigned int i = 0; i < cmd.length(); i++)
    {
        if (cmd[i] != '*')
        {
            cs = cs ^ cmd[i];
        }
    }
    cs &= 0xff;
    return cs;
}

/*
returns number of send chars
*/
int ODrive::send(const std::string msg, const std::string *funct_name)
{
    // flush in/out
    tcflush(fd, TCIOFLUSH);
    std::string msg_out = msg;
    msg_out += " *";
    msg_out += std::to_string(checksum(msg_out));
    msg_out += "\n";
    int w = write(fd, msg_out.c_str(), msg_out.size());
    if (w == 0)
    {
        RCLCPP_WARN(node->get_logger(), "%s send zero chars\n", funct_name->c_str());
    }
    return w;
}

/*
checks input against regex
*/
bool ODrive::check_double_output(const std::string msg, const std::string *funct_name)
{
    const std::string regex_ans = "^-?([0-9]+)(\\.){1}([0-9]+)[\\ ]{1}-?([0-9]+)(\\.){1}([0-9]+)$";
    if (std::regex_match(msg, std::regex(regex_ans)))
    {
        return true;
    }
    RCLCPP_WARN(node->get_logger(), "%s : regex caught wrong data: '%s'\n", funct_name->c_str(), msg.c_str());
    return false;
}

/*
receive message via uart, check for correctness and remove checksum
*/
std::string ODrive::receive(const std::string *funct_name)
{
    char read_buf[80];
    memset(read_buf, '\0', sizeof(read_buf)); // clean out buffer
    if (read(fd, read_buf, sizeof(read_buf) - 1) == 0)
    {
        RCLCPP_WARN(node->get_logger(), "%s received no data\n", funct_name->c_str());
        return "";
    }

    std::string ans = read_buf;

    if (funct_name->compare("setVelocity") == 0){
        return "velocity_ok";
    }

    // check checksum
    size_t i = ans.find('*');
    if (i == std::string::npos)
    {
        RCLCPP_WARN(node->get_logger(), "%s received no checksum", funct_name->c_str());
        return "";
    }
    std::string prefix = ans.substr(0, i);
    return prefix;
}

/*
send 0 Turn/s signal, returns -1 if nothing is send, 0 else
*/
int ODrive::stop()
{
    const std::string funct_name = "stop";
    std::string msg = "v 0 0";
    if (send(msg, &funct_name) > 0)
    {
        return 0;
    }
    return -1;
}

/*
send velocity, returns -1 if nothing is send, 0 else
*/
int ODrive::set_velocity(const int motor, const float velocity)
{
    const std::string funct_name = "setVelocity";
    std::string msg = "v ";
    msg += std::to_string(motor);
    msg += " ";
    msg += std::to_string(velocity);
    std::this_thread::sleep_for(200us);
    if (send(msg, &funct_name) > 0)
    {
        std::string ans = receive(&funct_name);
        return 0;
    }
    RCLCPP_WARN(node->get_logger(), "could not set velocity: %f", velocity);
    return -1;
}

/*
return tuple of position and velocity, returns both as -1 if request fails
*/
std::pair<float, float> ODrive::get_position_velocity(const int motor)
{
    const std::string funct_name = "getPosition_Velocity";
    std::string msg = "f ";
    msg += std::to_string(motor);
    if (send(msg, &funct_name) > 0)
    {
        std::string ans = receive(&funct_name);
        if (ans.size() > 0)
        {
            if (check_double_output(ans, &funct_name))
            {
                size_t i = ans.find(' ');
                try
                {
                    std::pair<float, float> output(std::stof(ans.substr(0, i)), std::stof(ans.substr(i + 1)));
                    return output;
                }
                catch (const std::invalid_argument &e)
                {
                    RCLCPP_WARN(node->get_logger(), "&s: received wrong data after regex check: %s\n", funct_name.c_str(), ans.c_str());
                }
            }
        }
    }
    return std::pair<float, float>(-1, -1);
}
