#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <ar_interfaces/action/movement.hpp>

class OdomClient : public BT::StatefulActionNode
{
public:
    using Movement = ar_interfaces::action::Movement;
    using GoalHandleMovement = rclcpp_action::ClientGoalHandle<Movement>;

    OdomClient(const std::string & name,
               const BT::NodeConfiguration & config,
               rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void cancel_goal();

    void goal_response_callback(const GoalHandleMovement::SharedPtr &);
    void result_callback(const GoalHandleMovement::WrappedResult &);
    void feedback_callback(
        GoalHandleMovement::SharedPtr,
        const std::shared_ptr<const Movement::Feedback>);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Movement>::SharedPtr odom_client_;
    GoalHandleMovement::SharedPtr goal_handle_;

    Movement::Goal goal_msg_;

    std::atomic_bool done_{false};
    std::atomic_bool success_{false};
};
