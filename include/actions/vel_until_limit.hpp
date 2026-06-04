#ifndef VEL_UNTIL_LIMIT_HPP
#define VEL_UNTIL_LIMIT_HPP

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qos_profiles.hpp"

using Byte = std_msgs::msg::UInt8;

class VelUntilLimit : public BT::StatefulActionNode
{
public:
    VelUntilLimit(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void limitSwitchCallback(const Byte::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<Byte>::SharedPtr byte_sub_;

    std::string color_;
    uint8_t byte_ = 0x00;
};

#endif