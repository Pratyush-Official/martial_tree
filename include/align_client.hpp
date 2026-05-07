#ifndef ALIGN_CLIENT_HPP
#define ALIGN_CLIENT_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "ar_interfaces/action/bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/odometry.hpp"

class AlignClient : public BT::StatefulActionNode
{
public:
    AlignClient(const std::string & name,
              const BT::NodeConfig & config,
              rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    bool done_flag{false};
    bool success{false};

    uint32_t start_time;

    using GoalHandleMovement = rclcpp_action::ClientGoalHandle<ar_interfaces::action::Bool>;
    using Bool = ar_interfaces::action::Bool;

    void cancel_goal();
    // void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    // void odometry_callback(const nav_msgs::msg::Odometry &msg);   

    void goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_);
    void result_callback(const GoalHandleMovement::WrappedResult &result);
    void feedback_callback(GoalHandleMovement::SharedPtr, const std::shared_ptr<const ar_interfaces::action::Bool::Feedback> feedback);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp_action::Client<ar_interfaces::action::Bool>::SharedPtr client_ptr_align;

    ar_interfaces::action::Bool::Goal goal_msg;
    //nav_msgs::msg::Odometry odom_msg;

    rclcpp_action::ClientGoalHandle<ar_interfaces::action::Bool>::SharedPtr goal_handle;
    std::vector<float> sick_data;
};

#endif
