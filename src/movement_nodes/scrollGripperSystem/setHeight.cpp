#include "movement_nodes/scrollGripperSystem/setHeight.hpp"
using namespace BT;

SetHeight::SetHeight(const std::string& name, 
    const NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
    client_ = node->create_client<MotorState>("/elev_vert_move");
}

BT::PortsList SetHeight::providedPorts()
{
    return { InputPort<float>("height") };
}
BT::NodeStatus SetHeight::onStart()
{
   
    response_received_ = false;
    call_succeeded_     = false;

    auto height = getInput<float>("height");
    if (!height) {
        RCLCPP_ERROR(node_->get_logger(), "[SetHeight] Missing port 'height'");
        return BT::NodeStatus::FAILURE;
    }
    height_state_ = height.value();

    if (!client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "[SetHeight] Service not ready, retrying next tick");
        return BT::NodeStatus::RUNNING; 
    }

    RCLCPP_INFO(node_->get_logger(), "[SetHeight] Sending request: %.2f", height_state_);

    auto request   = std::make_shared<MotorState::Request>();
    request->data  = true;
    request->state = height_state_;

    client_->async_send_request(
        request,
        [this](rclcpp::Client<MotorState>::SharedFuture future) {
            auto response      = future.get();
            call_succeeded_    = response->success;
            response_received_ = true;
            if (response->success)
                RCLCPP_INFO(node_->get_logger(), "[SetHeight] Succeeded");
            else
                RCLCPP_WARN(node_->get_logger(), "[SetHeight] Failed: %s", response->message.c_str());
        });

    return BT::NodeStatus::RUNNING;
}


BT::NodeStatus SetHeight::onRunning()
{
    if (!response_received_) return BT::NodeStatus::RUNNING;
    if (!call_succeeded_)    return BT::NodeStatus::FAILURE;
    RCLCPP_INFO(node_->get_logger(), "[SetHeight] :: Success");
    return BT::NodeStatus::SUCCESS;
}

void SetHeight::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[SetHeight] :: Halted");
}