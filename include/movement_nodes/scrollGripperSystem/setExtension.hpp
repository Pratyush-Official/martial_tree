#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "ar_interfaces/msg/gripper_command.hpp"
#include "ar_interfaces/msg/gripper_status.hpp"

using namespace BT;

class SetExtension : public StatefulActionNode
{
public:

    SetExtension(const std::string& name,const NodeConfiguration& config, rclcpp::Node::SharedPtr node);
       

    static PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void statusCallback(
        const ar_interfaces::msg::GripperStatus::SharedPtr msg);

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<
        ar_interfaces::msg::GripperCommand>::SharedPtr pub_;

    rclcpp::Subscription<
        ar_interfaces::msg::GripperStatus>::SharedPtr sub_;

    ar_interfaces::msg::GripperStatus status_;

    float target_length_;

    rclcpp::Time start_time_;
};