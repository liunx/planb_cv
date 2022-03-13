#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "robot_control.hpp"

namespace planb {
    int Robot::init(const char *dev, const int baud)
    {
        struct termios options;
        speed_t myBaud;
        int status;

        switch (baud)
        {
            case 9600:
                myBaud = B9600;
                break;
            case 115200:
                myBaud = B115200;
                break;
            default:
                return -2;
        }
        if ((fd_ = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
            return -1;
        fcntl(fd_, F_SETFL, O_RDWR);
        // Get and modify current options:
        tcgetattr(fd_, &options);
        cfmakeraw(&options);
        cfsetispeed(&options, myBaud);
        cfsetospeed(&options, myBaud);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 100; // Ten seconds (100 deciseconds)
        tcsetattr(fd_, TCSANOW, &options);
        ioctl(fd_, TIOCMGET, &status);
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        ioctl(fd_, TIOCMSET, &status);
        usleep(10000); // 10mS

        return 0;
    }

    void Robot::txData()
    {
        write(fd_, cmd_, sizeof(cmd_));
    }

    void Robot::forward()
    {
        if (cmd_[1] <= MAX_POWER_INDEX) {
            cmd_[1] += 1;
            txData();
        }
    }

    void Robot::setPower(uint8_t val)
    {
        if (power_ == val)
            return;

        if (val > MIN_POWER_INDEX && val <= MAX_POWER_INDEX) {
            cmd_[1] = val;
            txData();
            power_ = val;
        }
    }

    void Robot::backward()
    {
        if (cmd_[1] > MIN_POWER_INDEX) {
            cmd_[1] -= 1;
            txData();
        }
    }

    void Robot::turnLeft()
    {
        if (cmd_[0] > MIN_ANGLE_INDEX) {
            cmd_[0] -= 1;
            txData();
        }
    }

    void Robot::setAngle(uint8_t val)
    {
        if (val == angle_)
            return;
        if (val > MIN_ANGLE_INDEX && val <= MAX_ANGLE_INDEX) {
            cmd_[0] = val;
            txData();
            angle_ = val;
        }
    }

    void Robot::turnRight()
    {
        if (cmd_[0] <= MAX_ANGLE_INDEX) {
            cmd_[0] += 1;
            txData();
        }
    }

    void Robot::reset()
    {
        if (angle_ != 6 || power_ != 1)
        {
            cmd_[0] = 6;
            cmd_[1] = 1;
            txData();
            angle_ = 6;
            power_ = 1;
        }
    }

    void Robot::stop()
    {
        if (power_ != 1)
        {
            cmd_[1] = 1;
            txData();
            power_ = 1;
        }
    }
}