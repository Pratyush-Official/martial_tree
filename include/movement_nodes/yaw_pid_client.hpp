#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ar_interfaces/action/rotate_exec.hpp"

namespace BT
{

using RotateExec = ar_interfaces::action::RotateExec;
using GoalHandleRotate = rclcpp_action::ClientGoalHandle<RotateExec>;

class YawPIDClient : public BT::StatefulActionNode
{
public:
    YawPIDClient(const std::string &     name,
           const BT::NodeConfig &  config,
           rclcpp::Node::SharedPtr node);

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void onHalted()  override;

    static BT::PortsList providedPorts();

    
    rclcpp::Node::SharedPtr                       node_;
    rclcpp_action::Client<RotateExec>::SharedPtr  action_client_;
    GoalHandleRotate::SharedPtr                   goal_handle_;

    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;
    
    bool  goal_accepted_{false};
    bool  goal_rejected_{false};
    bool  goal_done_{false};
    bool  goal_succeeded_{false};

    
    float final_yaw_{0.0f};

    void reset();
};

} 