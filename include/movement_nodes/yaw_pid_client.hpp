#ifndef YAW_PID_CLIENT_HPP
#define YAW_PID_CLIENT_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "ar_interfaces/action/movement.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "qos_profiles.hpp"

class YawPIDClient : public BT::StatefulActionNode
{
public:
    YawPIDClient(const std::string & name,
              const BT::NodeConfig & config,
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
    void resetVel();

    void goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_);
    void result_callback(const GoalHandleMovement::WrappedResult &result);
    void feedback_callback(GoalHandleMovement::SharedPtr, const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<ar_interfaces::action::Movement>::SharedPtr client_ptr;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    ar_interfaces::action::Movement::Goal goal_msg;

    rclcpp_action::ClientGoalHandle<ar_interfaces::action::Movement>::SharedPtr goal_handle;
    std::vector<float> sick_data;
};

#endif
