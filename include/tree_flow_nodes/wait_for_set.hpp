#ifndef WAIT_FOR_SET_HPP
#define WAIT_FOR_SET_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/u_int8.hpp>

class WaitForSet : public BT::SyncActionNode
{
public:
    WaitForSet(
        const std::string &name,
        const BT::NodeConfig &config,
        rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr set_reset;

    void set_reset_subscriber_callback(std_msgs::msg::UInt8 msg);

    bool wait  = true;
    bool start = false;
};

#endif