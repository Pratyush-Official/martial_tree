#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "motor_interfaces/srv/motor_state.hpp"

using namespace BT;
using MotorState = motor_interfaces::srv::MotorState;

class GripperPose : public BT::StatefulActionNode
{
public:
        
    GripperPose(const std::string & name,
          const BT::NodeConfig & config,
          rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<MotorState>::SharedPtr client_;
    
    int angle_state_ {0}; // degree
    bool response_received_ {false};
    bool call_succeeded_    {false};
};

