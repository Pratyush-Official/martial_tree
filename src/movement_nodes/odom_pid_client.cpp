#include "movement_nodes/odom_pid_client.hpp"

using namespace std::placeholders;

OdomPIDClient::OdomPIDClient(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
    odom_client_ =
        rclcpp_action::create_client<Movement>(node_, "/movement_odom");

    RCLCPP_INFO(node_->get_logger(), "OdomPIDClient initialized");
}

BT::PortsList OdomPIDClient::providedPorts()
{
    return {
        BT::InputPort<float>("setpoint_odom_1"),
        BT::InputPort<float>("setpoint_odom_2"),
        BT::InputPort<float>("tolerance"),
        BT::InputPort<float>("timeout")
    };
}

BT::NodeStatus OdomPIDClient::onStart()
{
    done_ = false;
    success_ = false;

    if (!odom_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node_->get_logger(), "Odom action server not available");
        return BT::NodeStatus::FAILURE;
    }

    goal_msg_.setpoint_odom.resize(2);

    getInput("setpoint_odom_1", goal_msg_.setpoint_odom[0]);
    getInput("setpoint_odom_2", goal_msg_.setpoint_odom[1]);

    getInput("tolerance", goal_msg_.tolerance_odom);
    getInput("timeout", goal_msg_.timeout_odom);

    rclcpp_action::Client<Movement>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&OdomPIDClient::goal_response_callback, this, _1);
    options.feedback_callback =
        std::bind(&OdomPIDClient::feedback_callback, this, _1, _2);
    options.result_callback =
        std::bind(&OdomPIDClient::result_callback, this, _1);

    odom_client_->async_send_goal(goal_msg_, options);

    RCLCPP_INFO(node_->get_logger(), "OdomPIDClient goal sent");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomPIDClient::onRunning()
{
    if (!done_) {
        return BT::NodeStatus::RUNNING;
    }

    return success_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
}

void OdomPIDClient::onHalted()
{
    cancel_goal();
}

void OdomPIDClient::cancel_goal()
{
    if (goal_handle_) {
        odom_client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(node_->get_logger(), "OdomPIDClient goal canceled");
    }
}

void OdomPIDClient::goal_response_callback(
    const GoalHandleMovement::SharedPtr &handle)
{
    if (!handle) {
        RCLCPP_ERROR(node_->get_logger(), "OdomPIDClient goal rejected");
        done_ = true;
        success_ = false;
        return;
    }

    goal_handle_ = handle;
    RCLCPP_INFO(node_->get_logger(), "OdomPIDClient goal accepted");
}

void OdomPIDClient::result_callback(
    const GoalHandleMovement::WrappedResult &result)
{
    success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    done_ = true;

    RCLCPP_INFO(node_->get_logger(),
        "OdomPIDClient result: %s",
        success_ ? "SUCCESS" : "FAILURE");
}

void OdomPIDClient::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const Movement::Feedback>)
{
    
}
