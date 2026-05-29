#include <string>
#include <chrono>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "ar_interfaces/msg/gripper_command.hpp" //custom msg type
#include "ar_interfaces/msg/gripper_status.hpp"

class GripperPose : public BT::StatefulActionNode
{
public:
    GripperPose(const std::string& name, const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    int y_;

    std::chrono::steady_clock::time_point start_time_;

    bool timeout();
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<ar_interfaces::msg::GripperCommand>::SharedPtr pub_;
    rclcpp::Subscription<ar_interfaces::msg::GripperStatus>::SharedPtr sub_;
    ar_interfaces::msg::GripperStatus status_;

    void statusCallback(const ar_interfaces::msg::GripperStatus::SharedPtr msg);



};