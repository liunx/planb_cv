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
                   Robot &robot,
                   VisionDataStack &vdata)
            : BT::SyncActionNode(name, config),
            visionDataStack_(vdata),
            robot_(robot)
        {
        }

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts() { return {}; }

    private:
        VisionDataStack &visionDataStack_;
        Robot &robot_;
    };

    inline BT::Tree buildBehaviorTree(planb::Robot &robot,
                                      planb::VisionDataStack &vdata,
                                      std::string &xml)
    {
        BT::BehaviorTreeFactory factory;
        BT::NodeBuilder builder_A = [&robot, &vdata](const std::string &name,
                                                     const BT::NodeConfiguration &config)
        {
            return std::make_unique<planb::SyncActionNode>(name, config, robot, vdata);
        };

        factory.registerBuilder<planb::SyncActionNode>("SyncA", builder_A);
        auto tree = factory.createTreeFromText(xml);
        return tree;
    }
}

#endif