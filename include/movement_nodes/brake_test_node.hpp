#pragma once
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BrakeTest : public BT::StatefulActionNode
{
public:
    BrakeTest(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    void vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void brakeLoop();
    void publishVel(float vx, float vy);

    rclcpp::Node::SharedPtr node_ptr_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr                                      brake_timer_;

    // robot velocity from /robot_vel
    float robot_vel_x_ = 0.f;
    float robot_vel_y_ = 0.f;
    bool  vel_fresh_   = false;

    // internal state machine
    enum class TestPhase { IDLE, MOVING, BRAKING };
    TestPhase phase_ = TestPhase::IDLE;

    // move phase
    rclcpp::Time move_start_time_;
    float move_vx_          = 0.f;
    float move_vy_          = 0.f;
    int   move_duration_ms_ = 2000;

    // brake phase
    int  brake_ticks_left_ = 0;
    bool brake_active_     = false;

    // params from XML ports
    double brake_gain_x_      = 2.0;
    double brake_gain_y_      = 2.0;
    double max_brake_vel_     = 0.7;
    int    brake_duration_ms_ = 300;
    bool   y_direction_flip_  = true;
};