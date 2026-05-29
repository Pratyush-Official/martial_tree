
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tree_flow_nodes/conditions/check_for_S.hpp"
#include "behaviortree_cpp/bt_factory.h"


using namespace BT;
using namespace std::placeholders;

CheckForS::CheckForS(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node
):BT::SyncActionNode(name, config),node_(node)
{
}

BT::PortsList CheckForS::providedPorts()
{
    return{
        BT::InputPort<int>("next_node_state")  
    };
}
BT::NodeStatus CheckForS::tick()
{
    auto next_node_state = getInput<ScrollState>("next_node_state").value();
    if(next_node_state == static_cast<int>(ScrollState::S))
    {
        RCLCPP_INFO(node_->get_logger(),"[CheckForS]::scroll only yes");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
