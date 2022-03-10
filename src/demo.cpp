#include <iostream>
#include <string>
#include "control.hpp"

int main(int argc, char *argv[])
{
    planb::Robot rbt = planb::Robot();
    if (rbt.init("/dev/ttyS5", 9600) < 0) {
        std::cout << "Failed to init Robot controller!" << std::endl;
        return -1;
    }
    rbt.reset();
    sleep(1);
    rbt.forward();
    rbt.forward();
    rbt.forward();
    rbt.forward();
    sleep(1);
    rbt.stop();
    sleep(3);
    return 0;
}