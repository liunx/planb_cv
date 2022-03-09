#ifndef BEHAVIOR_TREE_NODES_HPP
#define BEHAVIOR_TREE_NODES_HPP
#include <mutex>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "data.hpp"

namespace planb {
    class SyncActionNode : public BT::SyncActionNode {

    public:
        // additional arguments passed to the constructor
        SyncActionNode(const std::string &name,
                   const BT::NodeConfiguration &config,
                   std::stack<VisionData> &visionStack,
                   std::mutex &lock)
            : BT::SyncActionNode(name, config),
            visionStack_(visionStack),
            lock_(lock)
        {
        }

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts() { return {}; }

    private:
        std::stack<VisionData> visionStack_;
        std::mutex &lock_;
    };
}

#endif