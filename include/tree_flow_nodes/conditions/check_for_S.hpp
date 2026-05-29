
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "tree_flow_nodes/decision_nodes/node_analysis.hpp"

using namespace std::placeholders;

class CheckForS: public BT::SyncActionNode
{
    public:
    CheckForS(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node);
    
        rclcpp::Node::SharedPtr node_;
        
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
    
    

};
