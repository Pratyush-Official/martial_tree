#include "actions/gripper_close.hpp"

GripperClose::GripperClose(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperClose"), "GripperClose::Ready..");
    byte_pub_ = node_->create_publisher<Byte>("/gripper/command", 10);
}

BT::PortsList GripperClose::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_GripperStatus")
    };
}

BT::NodeStatus GripperClose::tick()
{
    Byte msg;
    msg.data = static_cast<uint8_t>(GripperCommand::CLOSE);
    byte_pub_->publish(msg);

    setOutput<bool>("Op_GripperStatus", true);
    RCLCPP_INFO(rclcpp::get_logger("GripperClose"), "Gripper Closed");
    return BT::NodeStatus::SUCCESS;
}