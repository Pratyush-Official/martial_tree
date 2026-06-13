#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "motor_interfaces/srv/motor_state.hpp"

using MotorState = motor_interfaces::srv::MotorState;

class SetHeight : public BT::StatefulActionNode
{
public:
    SetHeight(const std::string& name,
              const BT::NodeConfig& config,
              rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<MotorState>::SharedPtr client_;

    float height_state_     {0.0f};
    bool  request_sent_      {false};
    bool  response_received_ {false};
    bool  call_succeeded_    {false};
};