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
              visionData_()
        {
        }
        BT::NodeStatus fetchData();
        BT::NodeStatus findTarget();
        BT::NodeStatus startMoving();
        BT::NodeStatus stopMoving();
    private:
        VisionDataStack &visionDataStack_;
        Robot &robot_;
        VisionData visionData_;
    };
}

#endif
