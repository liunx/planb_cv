#include "behavior_tree_nodes.hpp"
#include "action_nodes.hpp"

namespace planb {

    BT::NodeStatus SyncActionNode::tick()
    {
        BT::NodeStatus status = tick_functor_(*this);
        return status;
    }

    // class BehaviorTree {
    void BehaviorTree::registerSyncAction(const std::string &ID,
                                          const BT::SimpleActionNode::TickFunctor &tick_functor,
                                          BT::PortsList ports)
    {
        BT::NodeBuilder builder = [tick_functor, ID, this](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<SyncActionNode>(name, config, this->robot_, this->visionDataStack_, tick_functor);
        };

        BT::TreeNodeManifest manifest = {BT::NodeType::ACTION, ID, std::move(ports)};
        factory_.registerBuilder(manifest, builder);
    }

    void BehaviorTree::registerNodes()
    {
        static ActionNodes nodes = ActionNodes(visionDataStack_, robot_);
        factory_.registerSimpleAction("FetchData", std::bind(&ActionNodes::fetchData, &nodes));
        factory_.registerSimpleAction("FindTarget", std::bind(&ActionNodes::findTarget, &nodes));
        factory_.registerSimpleAction("StartMoving", std::bind(&ActionNodes::startMoving, &nodes));
        factory_.registerSimpleAction("StopMoving", std::bind(&ActionNodes::stopMoving, &nodes));
    }

    BT::Tree BehaviorTree::buildTree(const std::string &xml)
    {
        registerNodes();
        auto tree = factory_.createTreeFromFile(xml);
        return tree;
    }
    // } class BehaviorTree

}