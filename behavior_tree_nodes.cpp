#include "behavior_tree_nodes.hpp"

namespace planb {

    BT::NodeStatus SyncActionNode::tick()
    {
        return BT::NodeStatus::SUCCESS;
    }

}