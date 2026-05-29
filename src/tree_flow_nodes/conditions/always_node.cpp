
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tree_flow_nodes/conditions/always_node.hpp"
#include "behaviortree_cpp/bt_factory.h"


using namespace BT;
using namespace std::placeholders;

AlwaysNode::AlwaysNode(
    const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node
):BT::SyncActionNode(name, config),node_(node)
{

}
BT::PortsList AlwaysNode::providedPorts()
{
    return {        
        BT::InputPort<int>("state")
    };
}
BT::NodeStatus AlwaysNode::tick()
{
    auto state = getInput<int>("state").value();
    if(state == 1)
    {
        RCLCPP_INFO(node_->get_logger(),"[AlwaysNode]::Sucess");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(),"[AlwaysNode]::Failure");
    
    return BT::NodeStatus::FAILURE;
}
