#ifndef DATA_HPP
#define DATA_HPP
#include <opencv2/core/utility.hpp>
#include <stack>
#include <bitset>
#include "robot_control.hpp"

#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 360

#define BIT_ARUCO_DETECT 0
#define BIT_TRACKER_UPDATE 1


namespace planb {
    struct VisionData {
        int64_t timestamp;
        std::bitset<16> flagBits;
        cv::Rect2d boundingBox;
        VisionData() { flagBits = 0; }
    };

    struct VisionDataStack {
        std::mutex mlock;
        std::stack<VisionData> visionData;
    };
}

#endif