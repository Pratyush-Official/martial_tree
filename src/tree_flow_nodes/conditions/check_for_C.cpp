
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tree_flow_nodes/conditions/check_for_C.hpp"
#include "tree_flow_nodes/decision_nodes/node_analysis.hpp"
#include "behaviortree_cpp/bt_factory.h"


using namespace BT;
using namespace std::placeholders;

CheckForC::CheckForC(
    const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node
):BT::SyncActionNode(name, config),node_(node)
{

}
BT::PortsList CheckForC::providedPorts()
{
    return {        
        BT::InputPort<int>("next_node_state")
    };
}
BT::NodeStatus CheckForC::tick()
{
    auto next_node_state = getInput<int>("next_node_state").value();
    if(next_node_state == static_cast<int>(ScrollState::C))
    {
        RCLCPP_INFO(node_->get_logger(),"[CheckForC]::climb yes");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(),"[CheckForC]::climb no");
    
    return BT::NodeStatus::FAILURE;
}
