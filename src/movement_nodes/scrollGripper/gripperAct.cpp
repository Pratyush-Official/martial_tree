#include "movement_nodes/scrollGripper/gripperAct.hpp"
using namespace BT;

GripperAct::GripperAct(const std::string& name, 
    const NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
    client_ = node_->create_client<MotorState>("gripper_catch_set");
}

BT::PortsList GripperAct::providedPorts()
{
    return { InputPort<int>("state") };
}

BT::NodeStatus GripperAct::onStart()
{
    auto state = getInput<int>("state");
    if (!state)
    {
        RCLCPP_ERROR(node_->get_logger(), "[GripperAct] Missing required port 'state'");
        return BT::NodeStatus::FAILURE;
    }
    state_ = state.value();
    request_sent_      = false;
    response_received_ = false;
    call_succeeded_    = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperAct::onRunning()
{
    if (!request_sent_)
    {
        if (!client_->wait_for_service(std::chrono::seconds(0)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "[GripperAct] Waiting for service...");
            return BT::NodeStatus::RUNNING;
        }

        RCLCPP_INFO(node_->get_logger(), "[GripperAct] :: Actuating to state %d", state_);

        auto request   = std::make_shared<MotorState::Request>();
        request->data  = true;
        request->state = state_;

        client_->async_send_request(
            request,
            [this](rclcpp::Client<MotorState>::SharedFuture future)
            {
                auto response      = future.get();
                call_succeeded_    = response->success;
                response_received_ = true;
                if (response->success)
                    RCLCPP_INFO(node_->get_logger(), "[GripperAct] Succeeded: state %d", state_);
                else
                    RCLCPP_WARN(node_->get_logger(), "[GripperAct] Failed: %s", response->message.c_str());
            });

        request_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    if (!response_received_) return BT::NodeStatus::RUNNING;
    if (!call_succeeded_)    return BT::NodeStatus::FAILURE;
    RCLCPP_INFO(node_->get_logger(), "[GripperAct] :: Success");
    return BT::NodeStatus::SUCCESS;
}

void GripperAct::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[GripperAct] :: Halted");
}