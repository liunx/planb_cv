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

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";

//! [aruco_detect_markers_keys]
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video or image file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{si        |       | searched marker id }";
}
//! [aruco_detect_markers_keys]

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


int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    int camId = parser.get<int>("ci");
    int searchId = parser.get<int>("si");

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
        waitTime = 10;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    vector<Point2f> corner;
    Rect2d bbox;

    bool flag_detect = true;
    bool flag_tracker_update = false;

    inputVideo.set(CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(CAP_PROP_FRAME_HEIGHT, 360);

    while(inputVideo.grab()) {
        Mat frame;
        double ticks = (double)getTickCount();
        inputVideo.retrieve(frame);

        if (flag_detect) {
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
                    tracker = TrackerKCF::create();
                    tracker->init(frame, bbox);
                    flag_tracker_update = true;
                    flag_detect = false;
                }
            }
        }
        if (flag_tracker_update) {
            if (tracker->update(frame, bbox)) {
                rectangle(frame, bbox, Scalar(255, 0, 255), 2, 1);
            }
            else {
                printf("tracking error!!!\n");
                flag_tracker_update = false;
                flag_detect = true;
            }
        }

        double fps = getTickFrequency() / (getTickCount() - ticks);
        putText(frame, format("FPS = %.2f", fps),
                Point(10, 50), FONT_HERSHEY_SIMPLEX, 1.3, Scalar(0, 0, 255), 4);
        imshow("out", frame);

        printf("FPS: %.2f\n", fps);

        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

    return 0;
}
