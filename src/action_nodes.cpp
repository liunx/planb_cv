#include <iostream>
#include "action_nodes.hpp"
#include "data.hpp"

namespace planb 
{
    BT::NodeStatus ActionNodes::checkFlags()
    {
        if (visionData_.flagBits[BIT_TRACKER_UPDATE])
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus ActionNodes::checkStatus()
    {
        auto vdata = visionDataStack_.visionData;

        if (!vdata.empty()) {
            visionDataStack_.mlock.lock();
            visionData_ = vdata.top();
            vdata.pop();
            visionDataStack_.mlock.unlock();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus ActionNodes::sayHello()
    {
        std::cout << "say: Hello!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
}