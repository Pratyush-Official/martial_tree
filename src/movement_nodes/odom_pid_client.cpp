#include "movement_nodes/odom_pid_client.hpp"

OdomPIDClient::OdomPIDClient(const std::string & name,
           const BT::NodeConfig & config,
           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    action_client_ = rclcpp_action::create_client<MoveTwist>(node_, "move_twist");
}

BT::PortsList OdomPIDClient::providedPorts()
{
    return {
        BT::InputPort<float>("move_x", "X displacement from DecisionNode in mm"),
        BT::InputPort<float>("move_y", "Y displacement from DecisionNode in mm"),
    };
}

BT::NodeStatus OdomPIDClient::onStart()
{
    reset();

    float move_x_mm{0.0f}, move_y_mm{0.0f};
    getInput("move_x", move_x_mm);
    getInput("move_y", move_y_mm);

    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500)))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Odom PID Server not available – retrying");
        return BT::NodeStatus::FAILURE;
    }

    MoveTwist::Goal goal;
    goal.move_x = move_x_mm;
    goal.move_y = move_y_mm;

    auto options = rclcpp_action::Client<MoveTwist>::SendGoalOptions{};

    options.goal_response_callback =
        [this](const GoalHandleMove::SharedPtr & handle)
        {
            if (!handle) {
                RCLCPP_ERROR(node_->get_logger(), "Goal REJECTED");
                goal_rejected_ = true;
            } else {
                RCLCPP_INFO(node_->get_logger(), "Goal ACCEPTED");
                goal_handle_   = handle;
                goal_accepted_ = true;
            }
        };

    options.feedback_callback =
        [this](GoalHandleMove::SharedPtr,
               const std::shared_ptr<const MoveTwist::Feedback>)
        {
        };

    options.result_callback =
        [this](const GoalHandleMove::WrappedResult & result)
        {
            goal_done_      = true;
            goal_succeeded_ = result.result->success;
            final_avg_dist_ = result.result->final_avg_dist_m;
            RCLCPP_INFO(node_->get_logger(),
                        "[Move] Result: success=%s  final_dist=%.4f m",
                        goal_succeeded_ ? "true" : "false",
                        final_avg_dist_);
        };

    action_client_->async_send_goal(goal, options);

    RCLCPP_INFO(node_->get_logger(),
                "Goal sent move_x=%.1f mm  move_y=%.1f mm",
                move_x_mm, move_y_mm);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomPIDClient::onRunning()
{
    if (goal_rejected_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected = FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    if (goal_done_)
    {
        if (goal_succeeded_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Succeeded avg_dist=%.4f m", final_avg_dist_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "[Move] Server returned failure");
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void OdomPIDClient::onHalted()
{
    if (goal_handle_)
    {
        action_client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(node_->get_logger(), "[Move] Halted – goal cancelled.");
    }
    reset();
}

void OdomPIDClient::reset()
{
    goal_accepted_  = false;
    goal_rejected_  = false;
    goal_done_      = false;
    goal_succeeded_ = false;
    final_avg_dist_ = 0.0f;
    goal_handle_.reset();
}