#include <iostream>
#include "action_nodes.hpp"
#include "data.hpp"

namespace planb 
{
    BT::NodeStatus ActionNodes::checkBattery()
    {
        if (flagBits_[BIT_TRACKER_UPDATE])
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus ActionNodes::checkStatus()
    {
        auto vdata = visionDataStack_.visionData;

        if (!vdata.empty()) {
            visionDataStack_.mlock.lock();
            auto data = vdata.top();
            vdata.pop();
            visionDataStack_.mlock.unlock();
            flagBits_ = std::move(data.flagBits);
            std::cout << "Bits: " << flagBits_ << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
}