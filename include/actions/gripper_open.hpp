#ifndef GRIPPER_OPEN_HPP
#define GRIPPER_OPEN_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/u_int8.hpp"
#include "actions/gripper_command.hpp"

using Byte = std_msgs::msg::UInt8;

class GripperOpen : public BT::SyncActionNode
{
public:
    GripperOpen(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<Byte>::SharedPtr byte_pub_;
};

#endif