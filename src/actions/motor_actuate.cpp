#include "actions/motor_actuate.hpp"

MotorActuate::MotorActuate(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    RCLCPP_INFO(rclcpp::get_logger("MotorActuate"), "MotorActuate::Ready..");
    byte_pub_ = node_->create_publisher<Byte>("/gripper/command", 10);
}

BT::PortsList MotorActuate::providedPorts()
{
    return {
        // BT::OutputPort<bool>("Op_GripperStatus"),
        BT::InputPort<std::string>("MotorActuateEnum", "up or down")};
}

BT::NodeStatus MotorActuate::tick()
{
    auto res = getInput<std::string>("MotorActuateEnum");
    if (!res)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorActuate"), "Missing MotorActuateEnum port: %s", res.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    Byte msg;
    if (res.value() == "up")
        msg.data = static_cast<uint8_t>(GripperCommand::MOTORUP);
    else if (res.value() == "down")
        msg.data = static_cast<uint8_t>(GripperCommand::MOTORDOWN);
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorActuate"), "Unknown direction: %s", res.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    byte_pub_->publish(msg);
    // setOutput<bool>("Op_GripperStatus", true);
    RCLCPP_INFO(rclcpp::get_logger("MotorActuate"), "Gripper Closed - Motor %s", res.value().c_str());
    return BT::NodeStatus::SUCCESS;
}