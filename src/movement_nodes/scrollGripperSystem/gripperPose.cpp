#include "movement_nodes/scrollGripperSystem/gripperPose.hpp"
using namespace BT;

GripperPose::GripperPose(const std::string& name, 
    const NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
    client_ = node_->create_client<MotorState>("gripper_pitch_set");
}

BT::PortsList GripperPose::providedPorts()
{
    return { InputPort<float>("angle") };
}

BT::NodeStatus GripperPose::onStart()
{
    auto angle = getInput<float>("angle");
    if (!angle)
    {
        RCLCPP_ERROR(node_->get_logger(), "[GripperPose] Missing required port 'angle'");
        return BT::NodeStatus::FAILURE;
    }
    angle_state_ = angle.value();

    RCLCPP_INFO(node_->get_logger(), "[GripperPose] :: Setting angle to %.2f", angle_state_);

    auto request   = std::make_shared<MotorState::Request>();
    request->data  = true;
    request->state = angle_state_;

    client_->async_send_request(
        request,
        [this](rclcpp::Client<MotorState>::SharedFuture future)
        {
            auto response      = future.get();
            call_succeeded_    = response->success;
            response_received_ = true;
            if (response->success)
                RCLCPP_INFO(node_->get_logger(), "[GripperPose] Succeeded: %.2f", angle_state_);
            else
                RCLCPP_WARN(node_->get_logger(), "[GripperPose] Failed: %s", response->message.c_str());
        });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperPose::onRunning()
{
    if (!response_received_) return BT::NodeStatus::RUNNING;
    if (!call_succeeded_)    return BT::NodeStatus::FAILURE;
    RCLCPP_INFO(node_->get_logger(), "[GripperPose] :: Success");
    return BT::NodeStatus::SUCCESS;
}

void GripperPose::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[GripperPose] :: Halted");
}