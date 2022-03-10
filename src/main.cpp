/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <stack>
#include <bitset>
#include "robot_control.hpp"
#include "data.hpp"
#include "behavior_tree_nodes.hpp"

#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 360
#define WINDOW_NAME "Robot Control"

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";

const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL=16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video or image file, if omitted, input comes from camera }"
        "{xml      |       | Behavior tree xml file }"
        "{serial   | /dev/ttyS5 | Serial tty to open }"
        "{baud     | 9600  | Baudrate for serial }"
        "{ci       | 0     | Camera id if input doesn't come from video (-v) }"
        "{si       |       | searched marker id }";
}

vector<Point2f>
findMarker(int id, vector<int> &ids, vector<vector<Point2f>> &corners)
{
    for (auto i=0; i < ids.size(); i++) {
        if (ids[i] == id) {
            return corners.at(i);
        }
    }
    return {};
}

void robotController(BT::Tree &tree) noexcept
{
    // wait for video stream get ready
    this_thread::sleep_for(chrono::seconds(3));
    while (true) {
        tree.tickRoot();
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
}

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    int camId = parser.get<int>("ci");
    int searchId = parser.get<int>("si");
    int baud = parser.get<int>("baud");
    cv::String xml = parser.get<cv::String>("xml");
    cv::String serial = parser.get<cv::String>("serial");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary;
    if (parser.has("d")) {
        int dictionaryId = parser.get<int>("d");
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    }
    else {
        std::cerr << "Dictionary not specified" << std::endl;
        return 0;
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    Ptr<Tracker> tracker;
    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        //namedWindow(WINDOW_NAME, WINDOW_AUTOSIZE);
        //resizeWindow(WINDOW_NAME, VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
        waitTime = 10;
    } else {
        inputVideo.open(camId);
        inputVideo.set(CAP_PROP_FRAME_WIDTH, VIDEO_FRAME_WIDTH);
        inputVideo.set(CAP_PROP_FRAME_HEIGHT, VIDEO_FRAME_HEIGHT);
        waitTime = 10;
    }

    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    vector<Point2f> corner;
    Rect2d bbox;

    bool flagDetect = true;
    bool flagTrackerUpdate = false;
    // Robot controller
    planb::Robot robot = planb::Robot();
    if (robot.init(serial.c_str(), baud) < 0) {
        std::cout << "Failed to init Robot controller!" << std::endl;
        std::abort();
    }
    robot.reset();
    
    planb::VisionDataStack visionDataStack;
    planb::BehaviorTree btree = planb::BehaviorTree(visionDataStack, robot);
    auto tree = btree.buildTree(xml);
    std::thread robotThread(robotController,std::ref(tree));

    TickMeter tm;
    while(inputVideo.grab()) {
        Mat frame;
        tm.start();
        inputVideo.retrieve(frame);

        planb::VisionData data_ = planb::VisionData();
        if (flagDetect) {
            aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams, rejected);
            if (ids.size() > 0) {
                corner = findMarker(searchId, ids, corners);
                if (!corner.empty()) {
                    vector<float> xs = {corner.at(0).x, corner.at(1).x, corner.at(2).x, corner.at(3).x};
                    vector<float> ys = {corner.at(0).y, corner.at(1).y, corner.at(2).y, corner.at(3).y};
                    auto result_x = std::minmax_element(begin(xs), end(xs));
                    auto result_y = std::minmax_element(begin(ys), end(ys));
                    bbox = Rect(Point2f(*result_x.first, *result_y.first),
                                Point2f(*result_x.second, *result_y.second));
                    tracker = TrackerMOSSE::create();
                    tracker->init(frame, bbox);
                    flagTrackerUpdate = true;
                    flagDetect = false;
                    data_.flagBits.reset(BIT_ARUCO_DETECT);
                    data_.flagBits.set(BIT_TRACKER_UPDATE);
                }
            }
        }
        if (flagTrackerUpdate) {
            if (tracker->update(frame, bbox)) {
                rectangle(frame, bbox, Scalar(255, 0, 255), 2, 1);
                data_.boundingBox = std::move(bbox);
            }
            else {
                cout << "tracking error!!!" << endl;
                flagTrackerUpdate = false;
                flagDetect = true;
                data_.flagBits.set(BIT_ARUCO_DETECT);
                data_.flagBits.reset(BIT_TRACKER_UPDATE);
            }
        }
        data_.timestamp = getTickCount();
        visionDataStack.mlock.lock();
        visionDataStack.visionData.push(std::move(data_));
        visionDataStack.mlock.unlock();
        tm.stop();
        auto cost = tm.getTimeMilli();
        tm.reset();
#if 0
        putText(frame, format("Cost %.2f ms", cost),
                Point(10, 50), FONT_HERSHEY_SIMPLEX, 1.3, Scalar(0, 0, 255), 4);
        imshow(WINDOW_NAME, frame);
        printf("Cost %.2f ms\n", cost);
#endif

        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }
    robotThread.join();

    return 0;
}
