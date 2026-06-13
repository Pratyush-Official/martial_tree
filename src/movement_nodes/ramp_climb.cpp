#include "movement_nodes/ramp_climb.hpp"

RampClimb::RampClimb(const std::string &name, const BT::NodeConfig &config,
                               rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList RampClimb::providedPorts()
{
    return {
        BT::InputPort<std::string>("vel_axis"),
        BT::InputPort<double>("velocity"),
        BT::InputPort<std::string>("check_axis"),
        BT::InputPort<double>("level_threshold_deg"),
    };
}

BT::NodeStatus RampClimb::onStart()
{
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Float32MultiArray>(
        "/imu/data", sensor_qos(),
        std::bind(&RampClimb::imuCallback, this, std::placeholders::_1));

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RampClimb::onRunning()
{
    double angle;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (!imu_ready_) return BT::NodeStatus::RUNNING;
        auto check_axis = getInput<std::string>("check_axis").value();
        angle = (check_axis == "pitch") ? pitch_ : roll_;
    }

    double threshold_rad = getInput<double>("level_threshold_deg").value() * M_PI / 180.0;

    if (was_at_angle && std::abs(angle) < threshold_rad) {
        stopRobot();
        return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::Twist cmd;
    auto vel_axis = getInput<std::string>("vel_axis").value();
    double velocity = getInput<double>("velocity").value();

    if (vel_axis == "x") cmd.linear.x = velocity;
    else                  cmd.linear.y = velocity;

    cmd_pub_->publish(cmd);
    return BT::NodeStatus::RUNNING;
}

void RampClimb::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    roll_ = msg->data[0];
    pitch_ = msg->data[1];

    
    imu_ready_ = true;
}

void RampClimb::onHalted()
{
    stopRobot();
    imu_sub_.reset();
}

void RampClimb::stopRobot()
{
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
}