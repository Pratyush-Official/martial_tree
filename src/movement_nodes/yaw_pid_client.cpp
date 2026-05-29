#include "movement_nodes/yaw_pid_client.hpp"
#include <cmath>

namespace BT
{

YawPIDClient::YawPIDClient(const std::string &     name,
               const BT::NodeConfig &  config,
               rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    action_client_ = rclcpp_action::create_client<RotateExec>(node_, "rotate_exec");
}

BT::PortsList YawPIDClient::providedPorts()
{
    return {
        BT::InputPort<float>("rotate_state",
            "Target rotation in DEGREES from DecisionNode"),
        BT::OutputPort<float>("final_yaw",
            "Actual yaw reading when rotation completed"),
    };
}

BT::NodeStatus YawPIDClient::onStart()
{
    reset();

    float target_degrees{0.0f};
    if (!getInput("rotate_state", target_degrees))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[YawPIDClient] Missing required port 'rotate_state'");
        return BT::NodeStatus::FAILURE;
    }

    // convert degrees to radians
    float target_angle = target_degrees * M_PI / 180.0f;

    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500)))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[YawPIDClient] rotate_exec server not available – retrying");
        return BT::NodeStatus::FAILURE;
    }

    RotateExec::Goal goal;
    goal.target_angle = target_angle;

    auto options = rclcpp_action::Client<RotateExec>::SendGoalOptions{};

    options.goal_response_callback =
        [this](const GoalHandleRotate::SharedPtr & handle) {
            if (!handle) {
                RCLCPP_ERROR(node_->get_logger(), "[YawPIDClient] Goal REJECTED");
                goal_rejected_ = true;
            } else {
                RCLCPP_INFO(node_->get_logger(), "[YawPIDClient] Goal ACCEPTED");
                goal_handle_   = handle;
                goal_accepted_ = true;
            }
        };

    options.feedback_callback =
        [this](GoalHandleRotate::SharedPtr,
               const std::shared_ptr<const RotateExec::Feedback> fb) {
            RCLCPP_DEBUG(node_->get_logger(),
                         "[YawPIDClient] yaw_error=%.4f rad  current_yaw=%.4f rad",
                         fb->current_yaw_error, fb->current_yaw);
        };

    options.result_callback =
        [this](const GoalHandleRotate::WrappedResult & result) {
            goal_done_      = true;
            goal_succeeded_ = result.result->success;
            final_yaw_      = result.result->final_yaw;
            RCLCPP_INFO(node_->get_logger(),
                        "[YawPIDClient] Result: success=%s  final_yaw=%.4f rad",
                        goal_succeeded_ ? "true" : "false", final_yaw_);
        };

    action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(node_->get_logger(),
                "[YawPIDClient] Goal sent – target_degrees=%.1f  target_rad=%.4f",
                target_degrees, target_angle);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YawPIDClient::onRunning()
{
    if (goal_rejected_)
        return BT::NodeStatus::FAILURE;

    if (goal_done_)
    {
        setOutput("final_yaw", final_yaw_);
        return goal_succeeded_ ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void YawPIDClient::onHalted()
{
    if (goal_handle_)
    {
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    RCLCPP_WARN(node_->get_logger(), "[Rotate] Halted – goal cancelled.");
}

void YawPIDClient::reset()
{
    goal_accepted_  = false;
    goal_rejected_  = false;
    goal_done_      = false;
    goal_succeeded_ = false;
    final_yaw_      = 0.0f;
    goal_handle_.reset();
}

} // namespace BT