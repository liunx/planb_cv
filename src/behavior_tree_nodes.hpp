#ifndef BEHAVIOR_TREE_NODES_HPP
#define BEHAVIOR_TREE_NODES_HPP
#include <mutex>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "data.hpp"

namespace planb {
    class SyncActionNode : public BT::SyncActionNode {

    public:
        typedef std::function<BT::NodeStatus(BT::TreeNode&)> TickFunctor;
        // additional arguments passed to the constructor
        SyncActionNode(const std::string &name,
                   const BT::NodeConfiguration &config,
                   Robot &robot,
                   VisionDataStack &vdata,
                   TickFunctor tick_functor)
            : BT::SyncActionNode(name, config),
            visionDataStack_(vdata),
            robot_(robot),
            tick_functor_(std::move(tick_functor))
        {
        }

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts() { return {}; }

    private:
        VisionDataStack &visionDataStack_;
        Robot &robot_;
    protected:
        TickFunctor tick_functor_;
    };

    class BehaviorTree
    {
    public:
        BehaviorTree(VisionDataStack &visionDataStack, Robot &robot)
            : visionDataStack_(visionDataStack),
              robot_(robot),
              factory_(),
              tree_()
        {}

        void registerSyncAction(const std::string &ID,
                                const BT::SimpleActionNode::TickFunctor &tick_functor,
                                BT::PortsList ports={});

        BT::Tree buildTree(const std::string &xml);

    private:
        void registerNodes();
        VisionDataStack &visionDataStack_;
        Robot &robot_;
        BT::BehaviorTreeFactory factory_;
        BT::Tree tree_;
    };
}

#endif