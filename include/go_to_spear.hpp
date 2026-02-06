#ifndef GO_TO_SPEAR_HPP
#define GO_TO_SPEAR_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ar_interfaces/action/movement.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GoToSpear : public BT::StatefulActionNode
{
public:
    GoToSpear(const std::string & name,
              const BT::NodeConfiguration & config,
              rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    bool done_flag{false};
    bool success{false};
    uint32_t start_time;

    using GoalHandleMovement = rclcpp_action::ClientGoalHandle<ar_interfaces::action::Movement>;
    using Movement = ar_interfaces::action::Movement;

    void cancel_goal();
    void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry &msg);   

    void goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_);
    void result_callback(const GoalHandleMovement::WrappedResult &result);
    void feedback_callback(GoalHandleMovement::SharedPtr, const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp_action::Client<ar_interfaces::action::Movement>::SharedPtr client_ptr_spear;

    ar_interfaces::action::Movement::Goal goal_msg;
    nav_msgs::msg::Odometry odom_msg;

    rclcpp_action::ClientGoalHandle<ar_interfaces::action::Movement>::SharedPtr goal_handle;
    std::vector<float> sick_data;
};

#endif
