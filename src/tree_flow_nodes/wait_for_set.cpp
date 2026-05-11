#include "tree_flow_nodes/wait_for_set.hpp"   

WaitForSet::WaitForSet(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    set_reset = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "set_reset", qos_profile, std::bind(&WaitForSet::set_reset_subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"WaitForSet Initialized");
    wait = true;
}

BT::PortsList WaitForSet::providedPorts()
{
    return {
        };
}

 BT::NodeStatus WaitForSet::tick()
 {  
    if(!wait && start)
    {
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"WaitForSet::WAIT");

    return BT::NodeStatus::FAILURE;
 }

void WaitForSet::set_reset_subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1 && wait )
    {
        start = true;    
        wait = false;

    }
    else if (msg.data == 0)
    {
        start = false;    
        wait = true;
    }
}