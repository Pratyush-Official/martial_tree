#ifndef ZONE_CHANGE_HPP
#define ZONE_CHANGE_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

using std::string;

class ZoneChange : public BT::StatefulActionNode
{
public:
    ZoneChange(const string &name, const BT::NodeConfig &config, const rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // ROS node
    rclcpp::Node::SharedPtr node_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr team_color_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr set_reset_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr reset_to_martial_sub_;

    // Callbacks
    void team_color_subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
    void set_reset_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void reset_to_martial_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    // State
    string color_;
    string state_;
    uint8_t set_reset_val_ = 0x00;   // neutral default (not 0x00 or 0x01)
    uint8_t reset_to_martial_ = 0x00;
};

#endif