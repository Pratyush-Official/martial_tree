#include "actions/gripper_open.hpp"

GripperOpen::GripperOpen(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperOpen"), "GripperOpen::Ready..");
    byte_pub_ = node_->create_publisher<Byte>("/gripper/command", 10);
}

BT::PortsList GripperOpen::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_GripperStatus")
    };
}

BT::NodeStatus GripperOpen::tick()
{
    Byte msg;
    msg.data = static_cast<uint8_t>(GripperCommand::OPEN);
    byte_pub_->publish(msg);

    setOutput<bool>("Op_GripperStatus", true);
    RCLCPP_INFO(rclcpp::get_logger("GripperOpen"), "Gripper Closed");
    return BT::NodeStatus::SUCCESS;
}