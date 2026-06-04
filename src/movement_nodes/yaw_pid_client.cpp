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

    target_angle_ = target_degrees * static_cast<float>(M_PI) / 180.0f;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YawPIDClient::onRunning()
{
    if (!goal_sent_)
    {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[YawPIDClient] Waiting for rotate_exec server...");
            return BT::NodeStatus::RUNNING;
        }

        RotateExec::Goal goal;
        goal.target_angle = target_angle_;

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
                             "[YawPIDClient] yaw_error=%.4f  current_yaw=%.4f",
                             fb->current_yaw_error, fb->current_yaw);
            };

        options.result_callback =
            [this](const GoalHandleRotate::WrappedResult & wr) {
                goal_done_      = true;
                goal_succeeded_ = wr.result->success;
                final_yaw_      = wr.result->final_yaw;
                RCLCPP_INFO(node_->get_logger(),
                            "[YawPIDClient] Result: success=%s  final_yaw=%.4f",
                            goal_succeeded_ ? "true" : "false", final_yaw_);
            };

        action_client_->async_send_goal(goal, options);
        goal_sent_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[YawPIDClient] Goal sent – target_rad=%.4f", target_angle_);
        return BT::NodeStatus::RUNNING;
    }

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