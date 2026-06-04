#include "actions/vel_until_limit.hpp"

VelUntilLimit::VelUntilLimit(
    const std::string &name,
    const BT::NodeConfig &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    // Gripper is close initially
    // setOutput<bool>("Op_GripperStatus", true);
    RCLCPP_INFO(rclcpp::get_logger("VelUntilLimit"), "VelUntilLimit::Ready..");

    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
    byte_sub_ = node_->create_subscription<Byte>("/gripper/limit_switch", 10, std::bind(&VelUntilLimit::limitSwitchCallback, this, std::placeholders::_1));
    // byte_enum_pub = node_->create_publisher<Byte>("/gripper/command", 10);
}

BT::PortsList VelUntilLimit::providedPorts()
{
    return {
        // BT::OutputPort<bool>("Op_GripperStatus")
        BT::InputPort<std::string>("color")};
}
BT::NodeStatus VelUntilLimit::onStart()
{
    // setOutput<bool>("Op_GripperStatus", true); //True = closed
    // RCLCPP_INFO(rclcpp::get_logger("VelUntilLimit"),"Gripper Closed");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VelUntilLimit::onRunning()
{
    auto color_input = getInput<std::string>("color", color_);
    if (!color_input)
        return BT::NodeStatus::RUNNING;

    if (byte_ == 0x40)
        return BT::NodeStatus::SUCCESS;

    geometry_msgs::msg::Twist cmd_vel;
    if (color_ == "red")
    {
        cmd_vel.linear.y = 0.2;
    }
    else if (color_ == "blue")
    {
        cmd_vel.linear.y = -0.2;
    }
    cmd_pub_->publish(cmd_vel);

    return BT::NodeStatus::RUNNING;
}

void VelUntilLimit::onHalted() 
{
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
}

void VelUntilLimit::limitSwitchCallback(const Byte::SharedPtr msg)
{
    byte_ = msg->data;
}