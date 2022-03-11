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
        auto bbox = visionData_.boundingBox;
        auto area = bbox.width * bbox.height;

        // too close, won't moving
        if ((area / (VIDEO_FRAME_WIDTH * VIDEO_FRAME_HEIGHT)) > 0.5f) {
            robot_.stop();
            return BT::NodeStatus::FAILURE;
        }
        // turning target into center

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus ActionNodes::stopMoving()
    {
        robot_.stop();
        return BT::NodeStatus::SUCCESS;
    }
}