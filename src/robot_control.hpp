#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>

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
}

#endif