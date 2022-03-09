#ifndef DATA_HPP
#define DATA_HPP
#include <opencv2/core/utility.hpp>
#include <stack>
#include <bitset>

namespace planb {
    struct VisionData {
        int64_t timestamp;
        std::bitset<16> flagBits;
        cv::Rect2d boundingBox;
        VisionData() { flagBits = 0; }
    };
}


#endif