#ifndef SICK_PID_CLIENT_HPP
#define SICK_PID_CLIENT_HPP

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
#include "qos_profiles.hpp"
#include "get_roi_detections.hpp"
#include "yaw_setpoint.hpp"
class SickPIDClient : public BT::StatefulActionNode
{
public:
    SickPIDClient(const std::string & name,
              const BT::NodeConfiguration & config,
              rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    bool done_flag{false};
    bool success{false};

    float sp1;
    float sp2;
    float sp3;
    float sp4;

    uint32_t start_time;

    using GoalHandleMovement = rclcpp_action::ClientGoalHandle<ar_interfaces::action::Movement>;
    using Movement = ar_interfaces::action::Movement;

    // struct Setpoints{
    //     float sp1;
    //     float sp2;
    //     float sp3;
    //     float sp4;
    // }

    const std::vector<float> ROI1_SP = { 2100.0f, 8500.0f, 0.0f, 0.0f };
    const std::vector<float> ROI2_SP = { 2140.0f, 6000.0f, 0.0f, 0.0f };
    const std::vector<float> ROI3_SP = { 2135.0f, 4000.0f, 0.0f, 0.0f };

    const std::vector<float> RIGHT_ROI1_SP = { 9200.0f, 6500.0f, 0.0f, 0.0f };
    const std::vector<float> RIGHT_ROI2_SP = { 6450.0f, 6180.0f, 0.0f, 0.0f };
    const std::vector<float> RIGHT_ROI3_SP = { 4230.0f, 5580.0f, 0.0f, 0.0f };

    const std::vector<float> LEFT_ROI1_SP = { 2100.0f, 8500.0f, 0.0f, 0.0f };
    const std::vector<float> LEFT_ROI2_SP = { 2140.0f, 6000.0f, 0.0f, 0.0f };
    const std::vector<float> LEFT_ROI3_SP = { 2135.0f, 4000.0f, 0.0f, 0.0f };

    std::vector<float> true_setpoint;
    
    void cancel_goal();
    void sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry &msg);   
    void resetVel();

    void goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_);
    void result_callback(const GoalHandleMovement::WrappedResult &result);
    void feedback_callback(GoalHandleMovement::SharedPtr, const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback);

    void set_setpoints(const StageDetections & det, const int yaw_int_, std::string &turn_output);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sick_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp_action::Client<ar_interfaces::action::Movement>::SharedPtr client_ptr;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr setpoint_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    ar_interfaces::action::Movement::Goal goal_msg;
    nav_msgs::msg::Odometry odom_msg;
    bool check;
    bool first_stage_done;
    std::string under_roi;
    rclcpp_action::ClientGoalHandle<ar_interfaces::action::Movement>::SharedPtr goal_handle;
    std::vector<float> sick_data;
};

#endif
