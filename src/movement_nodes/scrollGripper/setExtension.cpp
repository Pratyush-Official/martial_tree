#include "movement_nodes/scrollGripper/setExtension.hpp"
using namespace BT;

SetExtension::SetExtension(const std::string& name, 
    const NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
    client_ = node_->create_client<MotorState>("/elev_horiz_move");
}

BT::PortsList SetExtension::providedPorts()
{
    return { InputPort<float>("length") };
}

BT::NodeStatus SetExtension::onStart()
{
    auto length = getInput<float>("length");
    if (!length)
    {
        RCLCPP_ERROR(node_->get_logger(), "[SetExtension] Missing required port 'length'");
        return BT::NodeStatus::FAILURE;
    }
    length_state_      = length.value();
    request_sent_      = false;
    response_received_ = false;
    call_succeeded_    = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetExtension::onRunning()
{
    if (!request_sent_)
    {
        if (!client_->wait_for_service(std::chrono::seconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[SetExtension] Waiting for service...");
            return BT::NodeStatus::RUNNING;
        }

        RCLCPP_INFO(node_->get_logger(), "[SetExtension] :: Setting length to %.2f", length_state_);

        auto request   = std::make_shared<MotorState::Request>();
        request->data  = true;
        request->state = length_state_;

        client_->async_send_request(
            request,
            [this](rclcpp::Client<MotorState>::SharedFuture future)
            {
                auto response      = future.get();
                call_succeeded_    = response->success;
                response_received_ = true;
                if (response->success)
                    RCLCPP_INFO(node_->get_logger(), "[SetExtension] Succeeded: %.2f", length_state_);
                else
                    RCLCPP_WARN(node_->get_logger(), "[SetExtension] Failed: %s", response->message.c_str());
            });

        request_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    if (!response_received_) return BT::NodeStatus::RUNNING;
    if (!call_succeeded_)    return BT::NodeStatus::FAILURE;
    RCLCPP_INFO(node_->get_logger(), "[SetExtension] :: Success");
    return BT::NodeStatus::SUCCESS;
}

void SetExtension::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[SetExtension] :: Halted");
}