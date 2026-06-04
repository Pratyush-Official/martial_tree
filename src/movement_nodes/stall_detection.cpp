#include "movement_nodes/stall_detection.hpp"

StallDetection::StallDetection(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(rclcpp::get_logger("StallDetection"), "StallDetection::Ready..");

    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
    wheel_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_feedback", sensor_qos(),
        std::bind(&StallDetection::wheelFeedbackCallback, this, std::placeholders::_1));
}

BT::PortsList StallDetection::providedPorts()
{
    return {
        BT::InputPort<std::string>("direction", "Left, Right, Front or Back"),
        BT::InputPort<float>("threshold", "Delta threshold to count as stall")
    };
}

BT::NodeStatus StallDetection::onStart()
{
    auto res_dir = getInput<std::string>("direction");
    auto res_th  = getInput<float>("threshold", threshold_);

    if (!res_dir || !res_th)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StallDetection"), "Missing port inputs");
        return BT::NodeStatus::FAILURE;
    }

    direction_ = res_dir.value();

    // Map direction to velocity and monitored indices
    geometry_msgs::msg::Twist cmd;
    if (direction_ == "Left")
    {
        cmd.linear.x = 0.0f;
        cmd.linear.y = -0.2f;
        monitor_indices_ = {0, 2};        // FL, RL
    }
    else if (direction_ == "Right")
    {
        cmd.linear.x = 0.0f;
        cmd.linear.y = 0.2f;
        monitor_indices_ = {1, 3};        // FR, RR
    }
    else if (direction_ == "Front")
    {
        cmd.linear.x = 0.2f;
        cmd.linear.y = 0.0f;
        monitor_indices_ = {0, 1, 2, 3};  // all 4
    }
    else if (direction_ == "Back")
    {
        cmd.linear.x = -0.2f;
        cmd.linear.y = 0.0f;
        monitor_indices_ = {0, 1, 2, 3};  // all 4
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("StallDetection"), 
                     "Unknown direction: %s", direction_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    vel_cmd_           = cmd;
    feedback_received_ = false;
    wheel_deltas_.fill(0.0f);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StallDetection::onRunning()
{
    cmd_pub_->publish(vel_cmd_);

    if (!feedback_received_)
        return BT::NodeStatus::RUNNING;

    for (const int idx : monitor_indices_)
    {
        if (std::abs(wheel_deltas_[idx]) < threshold_)
            return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(rclcpp::get_logger("StallDetection"),
                "Stall detected [%s] – deltas: [%.2f, %.2f, %.2f, %.2f]",
                direction_.c_str(),
                wheel_deltas_[0], wheel_deltas_[1], wheel_deltas_[2], wheel_deltas_[3]);

    cmd_pub_->publish(geometry_msgs::msg::Twist{});  // same as giving 0 to every velocity
    return BT::NodeStatus::SUCCESS;
}

void StallDetection::onHalted()
{
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    RCLCPP_WARN(rclcpp::get_logger("StallDetection"), "StallDetection halted");
}

void StallDetection::wheelFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 4)
    {
        RCLCPP_WARN(rclcpp::get_logger("StallDetection"),
                    "Expected 4 wheel values, got %zu", msg->data.size());
        return;
    }

    for (int i = 0; i < 4; i++)
        wheel_deltas_[i] = msg->data[i];

    feedback_received_ = true;
}