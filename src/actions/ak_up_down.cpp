#include "actions/ak_up_down.hpp"

InitClimb::InitClimb(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    RCLCPP_INFO(rclcpp::get_logger("InitClimb"), "InitClimb::Ready..");
    client_ = node_->create_client<InitClimbSrv>("/init_climb");
}

BT::PortsList InitClimb::providedPorts()
{
    return {
        BT::InputPort<int>("state", "0 or 1")
    };
}

BT::NodeStatus InitClimb::onStart()
{
    auto res = getInput<int>("state");
    if (!res)
    {
        RCLCPP_ERROR(rclcpp::get_logger("InitClimb"), "Missing state port: %s", res.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("InitClimb"), "Service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<InitClimbSrv::Request>();
    request->data  = true;
    request->state = static_cast<uint8_t>(res.value());

    future_ = client_->async_send_request(request).share();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus InitClimb::onRunning()
{
    if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        return BT::NodeStatus::RUNNING;

    auto response = future_.get();
    if (!response->success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("InitClimb"), "Init climb failed: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("InitClimb"), "Init climb success: %s", response->message.c_str());
    return BT::NodeStatus::SUCCESS;
}

void InitClimb::onHalted()
{
    RCLCPP_WARN(rclcpp::get_logger("InitClimb"), "InitClimb halted");
}