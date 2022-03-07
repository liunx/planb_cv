#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define MIN_ANGLE_INDEX 0
#define MAX_ANGLE_INDEX 12
#define MIN_POWER_INDEX 0
#define MAX_POWER_INDEX 8

namespace planb {
    class Robot {
    public:
        Robot()
        {
            cmd_[0] = 0x06;
            cmd_[1] = 0x01;
            cmd_[2] = 0xFF;
            cmd_[3] = 0xFF;
        }
        ~Robot() { close(fd_); }
        int init(const char *dev, const int baud);
        void reset();
        void forward();
        void setPower(uint8_t val);
        void backward();
        void turnLeft();
        void turnRight();
        void stop();
    private:
        void txData();
        int fd_;
        uint8_t cmd_[4];
    };

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
        if (val > MIN_POWER_INDEX && val <= MAX_POWER_INDEX) {
            cmd_[1] = val;
            txData();
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

    void Robot::turnRight()
    {
        if (cmd_[0] <= MAX_ANGLE_INDEX) {
            cmd_[0] += 1;
            txData();
        }
    }

    void Robot::reset()
    {
        cmd_[0] = 6;
        cmd_[1] = 1;
        txData();
    }

    void Robot::stop()
    {
        cmd_[1] = 1;
        txData();
    }
}

#endif