#ifndef ACTION_NODES_HPP
#define ACTION_NODES_HPP
#include "behaviortree_cpp_v3/bt_factory.h"
#include "data.hpp"

namespace planb 
{
    class ActionNodes
    {
    public:
        ActionNodes(VisionDataStack &visionDataStack, Robot &robot)
            : visionDataStack_(visionDataStack),
              robot_(robot),
              flagBits_(0)
        {}
        BT::NodeStatus checkBattery();
        BT::NodeStatus checkStatus();
    private:
        VisionDataStack &visionDataStack_;
        Robot &robot_;
        std::bitset<16> flagBits_;
    };
}

#endif
