#include "movement_nodes/climb.hpp"

#include <chrono>
#include <mutex>

using namespace BT;
using namespace std::chrono_literals;

Climb::Climb(const std::string & name,
             const BT::NodeConfig & config,
             rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    ak_up_client_     = node_->create_client<std_srvs::srv::SetBool>("ak_motor_up_execute");
    ak_down_client_   = node_->create_client<std_srvs::srv::SetBool>("ak_motor_down_exec");
    ak_stop_client_   = node_->create_client<std_srvs::srv::SetBool>("ak_motor_stop_execute");
    roll_up_client_   = node_->create_client<std_srvs::srv::SetBool>("ak_motor_roll_up");
    roll_down_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_roll_down");
}

BT::PortsList Climb::providedPorts()
{
    return {
        BT::InputPort<int>("up_down"),
        BT::InputPort<int>("next_climb_state", "Used for preset roll after retract")
    };
}

BT::NodeStatus Climb::onStart()
{
    auto climb = getInput<int>("up_down");
    if (!climb)
    {
        RCLCPP_ERROR(node_->get_logger(), "[Climb] Missing required port 'up_down'");
        return BT::NodeStatus::FAILURE;
    }

    climb_state_ = climb.value();

    auto client = selectClient(climb_state_);
    if (!client)
    {
        RCLCPP_ERROR(node_->get_logger(), "[Climb] Unknown climb state: %d", climb_state_);
        return BT::NodeStatus::FAILURE;
    }

    if (!client->wait_for_service(500ms))
    {
        RCLCPP_ERROR(node_->get_logger(), "[Climb] Service not available for state: %d", climb_state_);
        return BT::NodeStatus::FAILURE;
    }

    {
        std::lock_guard<std::mutex> lock(result_);
        response_received_ = false;
        call_succeeded_    = false;
        preset_sent_       = false;
        preset_succeded_   = false;
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    RCLCPP_INFO(node_->get_logger(), "[Climb] Sending command: %d", climb_state_);

    client->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
                RCLCPP_INFO(node_->get_logger(), "[Climb] Succeeded: %d", climb_state_);
            else
                RCLCPP_WARN(node_->get_logger(), "[Climb] Failed: %d", climb_state_);

            std::lock_guard<std::mutex> lock(result_);
            call_succeeded_    = response->success;
            response_received_ = true;
        });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Climb::onRunning()
{
    std::lock_guard<std::mutex> lock(result_);

    if (!response_received_)
        return BT::NodeStatus::RUNNING;

    if (!call_succeeded_)
        return BT::NodeStatus::FAILURE;

    // if (climb_state_ == -1 && !preset_sent_)
    // {
    //     sendPreset();
    //     preset_sent_ = true;
    // }
    // if(preset_sent_)
    // {
    //     if (!preset_succeded_)
    //     {
    //         RCLCPP_INFO(node_->get_logger(), "[Climb] waiting for preset:");
    //         return BT::NodeStatus::RUNNING;
    //     }
    // }
    
    RCLCPP_INFO(node_->get_logger(), "[Climb] retracted and climbed:");

    return BT::NodeStatus::SUCCESS;
}

void Climb::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[Climb] Halted: %d", climb_state_);

    std::lock_guard<std::mutex> lock(result_);
    response_received_ = false;
    call_succeeded_    = false;
    preset_sent_       = false;
}

void Climb::sendPreset()
{
    auto next = getInput<int>("next_climb_state");
    if (!next)
    {
        RCLCPP_WARN(node_->get_logger(), "[Climb] No next_climb_state — skipping preset");
        return;
    }

    int next_state = next.value();
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;
    const char* svc_name;

    if (next_state == static_cast<int>(ClimbState::UP))
    {
        client   = roll_up_client_;
        svc_name = "ak_motor_roll_up";
    }
    else if (next_state == static_cast<int>(ClimbState::DOWN))
    {
        client   = roll_down_client_;
        svc_name = "ak_motor_roll_down";
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "[Climb] Unknown next_climb_state: %d — skipping preset", next_state);
        return;
    }

    if (!client->wait_for_service(200ms))
    {
        RCLCPP_WARN(node_->get_logger(), "[Climb] Preset service '%s' not available — skipping", svc_name);
        return;
    }

    auto req  = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;

    RCLCPP_INFO(node_->get_logger(), "[Climb] Sending preset: %s", svc_name);

    client->async_send_request(
        req,
        [this, svc_name](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(node_->get_logger(),
                        "[Climb] Preset '%s' response: %s",
                        svc_name, response->message.c_str());
            std::lock_guard<std::mutex> lock(result_);
            preset_succeded_    = response->success;
        });
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
Climb::selectClient(int state)
{
    if (state ==  1) return ak_up_client_;
    if (state == -1) return ak_down_client_;
    if (state ==  0) return ak_stop_client_;
    return 0;  
}