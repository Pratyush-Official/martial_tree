#include "tree_flow_nodes/conditions/is_inside_forest.hpp"   

isInsideForest::isInsideForest(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_inside_forest", 10,
        std::bind(&isInsideForest::subscriber_callback,this,std::placeholders::_1));
    inside = true;
    RCLCPP_INFO(node_ptr_->get_logger(),"isInsideForest::Ready");

}

BT::NodeStatus isInsideForest::tick()
{  
    if(inside)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"isInsideForest::yes");
        // inside = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void isInsideForest::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    inside = false;
    if(msg.data == 1)
        inside = true;  
}
