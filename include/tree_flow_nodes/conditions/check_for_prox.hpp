#pragma once
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "tree_flow_nodes/decision_nodes/decision_node.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CheckForProx : public BT::SyncActionNode
{
public:
    CheckForProx(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void front_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg);
    void rear_subscriber_callback (const std_msgs::msg::Byte::SharedPtr msg);
    void vel_callback             (const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void brakeLoop();
    void publishVel(float vx, float vy);

    rclcpp::Node::SharedPtr node_ptr_;

    // subscriptions — same style as old hpp
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Byte>>             front_subscription_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Byte>>             rear_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      vel_sub_;

    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                cmd_vel_pub_;

    // brake timer
    rclcpp::TimerBase::SharedPtr brake_timer_;

    // prox state — same as old
    bool front_prox_ = false;
    bool rear_prox_  = false;

    // velocity state — new
    float robot_vel_x_ = 0.f;
    float robot_vel_y_ = 0.f;
    bool  vel_fresh_   = false;

    // brake state — new
    bool brake_active_     = false;
    int  brake_ticks_left_ = 0;

    // params — new, set from BT ports each tick
    double brake_gain_x_      = 2.0;
    double brake_gain_y_      = 2.0;
    double max_brake_vel_     = 0.7;
    int    brake_duration_ms_ = 300;
    bool   y_direction_flip_  = true;
};