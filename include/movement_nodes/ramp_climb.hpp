#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include "qos_profiles.hpp"

class RampClimb : public BT::StatefulActionNode
{
public:
    RampClimb(const std::string &name, const BT::NodeConfig &config,
                   rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void stopRobot();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::mutex imu_mutex_;
    double roll_  = 0.0;
    double pitch_ = 0.0;
    bool imu_ready_ = false;
    bool was_at_angle = false;
};