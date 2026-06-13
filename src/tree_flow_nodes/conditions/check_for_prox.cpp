#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tree_flow_nodes/conditions/check_for_prox.hpp"
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
using namespace std::placeholders;

CheckForProx::CheckForProx(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_ptr_(node)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // same as old
    front_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Byte>(
        "/prox_front_received", qos_profile,
        std::bind(&CheckForProx::front_subscriber_callback, this, std::placeholders::_1));

    rear_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Byte>(
        "/prox_rear_received", qos_profile,
        std::bind(&CheckForProx::rear_subscriber_callback, this, std::placeholders::_1));

    // new additions
    vel_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot_vel", qos_profile,
        std::bind(&CheckForProx::vel_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
        "/brake/cmd_vel", qos_profile);

    brake_timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CheckForProx::brakeLoop, this));

    RCLCPP_INFO(node_ptr_->get_logger(), "[CheckForProx] Initialized.");
}

// ---------------------------------------------------------------
BT::PortsList CheckForProx::providedPorts()
{
    return {
        BT::InputPort<int>   ("climb_state"),
        BT::InputPort<double>("brake_gain_x"),
        BT::InputPort<double>("brake_gain_y"),
        BT::InputPort<double>("max_brake_vel"),
        BT::InputPort<int>   ("brake_duration_ms"),
        BT::InputPort<bool>  ("y_direction_flip"),
    };
}

// ---------------------------------------------------------------
BT::NodeStatus CheckForProx::tick()
{
    // read params from XML ports — falls back to member defaults if not in XML
    brake_gain_x_      = getInput<double>("brake_gain_x").value_or(brake_gain_x_);
    brake_gain_y_      = getInput<double>("brake_gain_y").value_or(brake_gain_y_);
    max_brake_vel_     = getInput<double>("max_brake_vel").value_or(max_brake_vel_);
    brake_duration_ms_ = getInput<int>   ("brake_duration_ms").value_or(brake_duration_ms_);
    y_direction_flip_  = getInput<bool>  ("y_direction_flip").value_or(y_direction_flip_);

    auto climb_state_expected = getInput<int>("climb_state");
    if (!climb_state_expected)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(),
            "[CheckForProx] Missing 'climb_state' input");
        return BT::NodeStatus::FAILURE;
    }

    auto climb_state = climb_state_expected.value();

    // UP — same logic as old, service call replaced with brake engage
    if (climb_state == static_cast<int>(ClimbState::UP))
    {
        if (front_prox_)
        {
            RCLCPP_INFO(node_ptr_->get_logger(), "[CheckForProx]:: detected front");
            brake_ticks_left_ = brake_duration_ms_ / 10;
            brake_active_     = true;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_INFO(node_ptr_->get_logger(), "[CheckForProx]::not front");
            return BT::NodeStatus::FAILURE;
        }
    }
    // DOWN — same logic as old
    else if (climb_state == static_cast<int>(ClimbState::DOWN))
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[CheckForProx]::rear");
        if (rear_prox_)
        {
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            brake_ticks_left_ = brake_duration_ms_ / 10;
            brake_active_     = true;
            return BT::NodeStatus::SUCCESS;
        }
    }

    RCLCPP_ERROR(node_ptr_->get_logger(), "[CheckForProx] Unknown ClimbState");
    return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------
void CheckForProx::brakeLoop()
{
    if (!brake_active_) return;

    if (brake_ticks_left_ <= 0)
    {
        brake_active_ = false;
        publishVel(0.f, 0.f);
        RCLCPP_INFO(node_ptr_->get_logger(), "[CheckForProx] Brake done, zeroed.");
        return;
    }

    if (!vel_fresh_) return;

    double y_sign = y_direction_flip_ ? 1.0 : -1.0;

    float vx_cmd = static_cast<float>(-brake_gain_x_ * robot_vel_x_);
    float vy_cmd = static_cast<float>( y_sign * brake_gain_y_ * robot_vel_y_);

    auto clamp = [](float v, float lim) {
        return v >  lim ?  lim :
               v < -lim ? -lim : v;
    };
    vx_cmd = clamp(vx_cmd, static_cast<float>(max_brake_vel_));
    vy_cmd = clamp(vy_cmd, static_cast<float>(max_brake_vel_));

    publishVel(vx_cmd, vy_cmd);

    RCLCPP_INFO(node_ptr_->get_logger(),
        "[CheckForProx] vel=(%.3f,%.3f) cmd=(%.3f,%.3f) ticks_left=%d",
        robot_vel_x_, robot_vel_y_, vx_cmd, vy_cmd, brake_ticks_left_);

    --brake_ticks_left_;
}

// ---------------------------------------------------------------
void CheckForProx::publishVel(float vx, float vy)
{
    geometry_msgs::msg::Twist t;
    t.linear.x  = vx;
    t.linear.y  = vy;
    t.angular.z = 0.0;
    cmd_vel_pub_->publish(t);
}

void CheckForProx::vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) return;
    robot_vel_x_ = msg->data[0];
    robot_vel_y_ = msg->data[1];
    vel_fresh_   = true;
}

// same as old
void CheckForProx::front_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    if (msg->data == 1)
        front_prox_ = true;
    else
        front_prox_ = false;
}

void CheckForProx::rear_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    if (msg->data == 1)
        rear_prox_ = true;
    else
        rear_prox_ = false;
}

// ---------------------------------------------------------------
// same as old
namespace BT {
    template <> inline ClimbState convertFromString(StringView str)
    {
        if (str == "UP")   return ClimbState::UP;
        if (str == "DOWN") return ClimbState::DOWN;
        throw RuntimeError(std::string("Invalid ClimbState: ") + str.data());
    }
    template <> inline std::string toStr<ClimbState>(const ClimbState & val)
    {
        return val == ClimbState::UP ? "UP" : "DOWN";
    }
}