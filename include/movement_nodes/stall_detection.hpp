#ifndef STALL_DETECTION_HPP
#define STALL_DETECTION_HPP

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qos_profiles.hpp"

class StallDetection : public BT::StatefulActionNode
{
public:
    StallDetection(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void wheelFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::array<float, 4> wheel_deltas_ = {0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<int> monitor_indices_;
    geometry_msgs::msg::Twist vel_cmd_;
    bool feedback_received_ = false;

    std::string direction_;
    float threshold_ = 0.0f;
};

#endif