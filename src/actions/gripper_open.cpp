#include "actions/gripper_open.hpp"

GripperOpen::GripperOpen(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Gripper is open initially
    setOutput<bool>("Op_GripperStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("GripperOpen"),"GripperOpen::Ready..");
}

BT::PortsList GripperOpen::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_GripperStatus")
};   
}
BT::NodeStatus GripperOpen::tick() 
{
    setOutput<bool>("Op_GripperStatus", false); //False = open
    RCLCPP_INFO(rclcpp::get_logger("GripperOpen"),"Gripper Opened");
    return BT::NodeStatus::SUCCESS;
}