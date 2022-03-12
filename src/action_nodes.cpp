#include <iostream>
#include "action_nodes.hpp"
#include "data.hpp"

namespace planb 
{
    BT::NodeStatus ActionNodes::fetchData()
    {
        auto vdata = &visionDataStack_.visionData;

        if (vdata->empty()) {
            return BT::NodeStatus::FAILURE;
        }
        visionDataStack_.mlock.lock();
        visionData_ = vdata->top();
        vdata->pop();
        visionDataStack_.mlock.unlock();
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus ActionNodes::findTarget()
    {
        if (!visionData_.flagBits[BIT_TRACKER_UPDATE])
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus ActionNodes::startMoving()
    {
        auto padding = 30;
        auto bbox = visionData_.boundingBox;
        auto areaBox = bbox.width * bbox.height;
        auto areaFrame = VIDEO_FRAME_WIDTH * VIDEO_FRAME_HEIGHT;
        auto centerX = VIDEO_FRAME_WIDTH / 2;

        // too close, won't moving
        if ((areaBox / areaFrame) > 0.5f) {
            robot_.stop();
            return BT::NodeStatus::FAILURE;
        }
        // turning target into center
        if (centerX > (bbox.x + bbox.width) + padding)
            robot_.setAngle(4);
        else if (centerX < bbox.x - padding)
            robot_.setAngle(8);
        else
            robot_.setAngle(6);

        robot_.setPower(3);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus ActionNodes::stopMoving()
    {
        robot_.stop();
        return BT::NodeStatus::SUCCESS;
    }
}