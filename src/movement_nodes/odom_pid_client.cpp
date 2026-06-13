#include "movement_nodes/odom_pid_client.hpp"

OdomPIDClient::OdomPIDClient(const std::string & name,
           const BT::NodeConfig & config,
           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    action_client_ = rclcpp_action::create_client<MoveTwist>(node_, "move_twist");
    setpoint_pub = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/setpoint_yaw",cmd_vel_qos());
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
}

BT::PortsList OdomPIDClient::providedPorts()
{
    return {
        BT::InputPort<std::vector<float>>("setpoint", "X and Y displacement in mm")
    };
}

BT::NodeStatus OdomPIDClient::onStart()
{
    reset();

    auto res = getInput<std::vector<float>>("setpoint");
    if (!res || res.value().size() < 2)
    {
        RCLCPP_ERROR(node_->get_logger(), "[OdomPIDClient] Invalid setpoint port");
        return BT::NodeStatus::FAILURE;
    }

    move_x_ = res.value()[0];
    move_y_ = res.value()[1];

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomPIDClient::onRunning()
{
    setpoint_.data.clear(); //This is for resetting the yaw_setpoint byte
    setpoint_.data.push_back(0.0);
    setpoint_.data.push_back(RESET_BYTE); 
    setpoint_pub->publish(setpoint_);

    if (!goal_sent_)
    {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "[OdomPIDClient] Waiting for move_twist server...");
            return BT::NodeStatus::RUNNING;
        }

        MoveTwist::Goal goal;
        goal.move_x = move_x_;
        goal.move_y = move_y_;

        auto options = rclcpp_action::Client<MoveTwist>::SendGoalOptions{};

        options.goal_response_callback =
            [this](const GoalHandleMove::SharedPtr & handle) {
                if (!handle) {
                    RCLCPP_ERROR(node_->get_logger(), "[Move] Goal REJECTED");
                    goal_rejected_ = true;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[Move] Goal ACCEPTED");
                    goal_handle_   = handle;
                    goal_accepted_ = true;
                }
            };

        options.feedback_callback =
            [this](GoalHandleMove::SharedPtr,
                   const std::shared_ptr<const MoveTwist::Feedback>) {};

        options.result_callback =
            [this](const GoalHandleMove::WrappedResult & result) {
                goal_done_      = true;
                goal_succeeded_ = result.result->success;
                final_avg_dist_ = result.result->final_avg_dist_m;
                RCLCPP_INFO(node_->get_logger(),
                            "[Move] Result: success=%s  final_dist=%.4f m",
                            goal_succeeded_ ? "true" : "false", final_avg_dist_);
            };

        action_client_->async_send_goal(goal, options);
        goal_sent_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[Move] Goal sent – move_x=%.1f mm  move_y=%.1f mm",
                    move_x_, move_y_);
        return BT::NodeStatus::RUNNING;
    }

    if (goal_rejected_)
    {
        RCLCPP_ERROR(node_->get_logger(), "[Move] Goal rejected – FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    if (goal_done_)
    {
        if (goal_succeeded_) {
            RCLCPP_INFO(node_->get_logger(),
                        "[Move] Succeeded avg_dist=%.4f m", final_avg_dist_);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_WARN(node_->get_logger(), "[Move] Server returned failure");
        return BT::NodeStatus::FAILURE;
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

    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);

    reset();
}

void OdomPIDClient::reset()
{
    goal_sent_      = false; 
    goal_accepted_  = false;
    goal_rejected_  = false;
    goal_done_      = false;
    goal_succeeded_ = false;
    final_avg_dist_ = 0.0f;
    move_x_         = 0.0f;   
    move_y_         = 0.0f;   
    goal_handle_.reset();
}