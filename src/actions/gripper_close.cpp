#include "actions/gripper_close.hpp"

GripperClose::GripperClose(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Gripper is open initially
    setOutput<bool>("Op_GripperStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("GripperClose"),"GripperClose::Ready..");
}

BT::PortsList GripperClose::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_GripperStatus")
};   
}
BT::NodeStatus GripperClose::tick() 
{
    setOutput<bool>("Op_GripperStatus", true); //True = closed
    RCLCPP_INFO(rclcpp::get_logger("GripperClose"),"Gripper Closed");
    return BT::NodeStatus::SUCCESS;
}