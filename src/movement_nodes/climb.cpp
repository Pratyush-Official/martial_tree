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
    ak_up_client_   = node_->create_client<std_srvs::srv::SetBool>("ak_motor_up_execute");
    ak_down_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_down_exec");
    ak_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_stop_execute");
    roll_up_client_ = node_->create_client<motor_interfaces::srv::MotorState>("ak_motor_roll_up");
}

BT::PortsList Climb::providedPorts()
{
    return {
        BT::InputPort<int>("up_down"),
        BT::InputPort<int>("next_climb_state", "Used for preset roll after retract"),
        BT::InputPort<int>("climb_state",1, "Current climb direction: UP or DOWN")
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

    if (!selectClient(climb_state_))
    {
        RCLCPP_ERROR(node_->get_logger(), "[Climb] Unknown climb state: %d", climb_state_);
        return BT::NodeStatus::FAILURE;
    }

    // Reset all state — no service call here
    std::lock_guard<std::mutex> lock(result_);
    response_received_ = false;
    call_succeeded_    = false;
    request_sent_      = false;  

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Climb::onRunning()
{
    {
        std::lock_guard<std::mutex> lock(result_);
        if (!request_sent_)
        {
            auto client = selectClient(climb_state_);

            if (!client->wait_for_service(500ms))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[Climb] Service not available for state: %d", climb_state_);
                return BT::NodeStatus::FAILURE;
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

            request_sent_ = true;
        }
    }

    // Subsequent ticks: poll for completion
    std::lock_guard<std::mutex> lock(result_);

    if (!response_received_)
        return BT::NodeStatus::RUNNING;

    if (!call_succeeded_)
        return BT::NodeStatus::FAILURE;

    RCLCPP_INFO(node_->get_logger(), "[Climb] Climb complete");
    return BT::NodeStatus::SUCCESS;
}

void Climb::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[Climb] Halted: %d", climb_state_);

    std::lock_guard<std::mutex> lock(result_);
    response_received_ = false;
    call_succeeded_    = false;
    request_sent_      = false;
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
Climb::selectClient(int state)
{
    if (state ==  1) return ak_up_client_;
    if (state == -1) return ak_down_client_;
    // if (state ==  0) return ak_stop_client_;
    return nullptr;
}