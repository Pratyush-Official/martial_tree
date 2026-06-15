#include "movement_nodes/roll.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace BT
{

Roll::Roll(const std::string &     name,
           const BT::NodeConfig &  config,
           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // [Fix] Declare parameters with default values so they exist on the server
    if (!node_->has_parameter("roll.roll_up_angle")) {
        node_->declare_parameter("roll.roll_up_angle", 0.0);
    }
    if (!node_->has_parameter("roll.roll_down_angle")) {
        node_->declare_parameter("roll.roll_down_angle", 0.0);
    }

    roll_up_client_ = node_->create_client<motor_interfaces::srv::MotorState>("ak_motor_roll_up");

    prox_front_sub_ = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_front_received", qos_profile,
        [this](const std_msgs::msg::Byte::SharedPtr msg) {
            front_edge_ = (msg->data == 1);
        });

    prox_rear_sub_ = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_rear_received", qos_profile,
        [this](const std_msgs::msg::Byte::SharedPtr msg) {
            rear_edge_ = (msg->data == 1);
        });
}

BT::PortsList Roll::providedPorts()
{
    return {
        BT::InputPort<int>("climb_state", "Climb state: UP or DOWN"),
    };
}

BT::NodeStatus Roll::onStart()
{
    reset(); 

    auto climb = getInput<int>("climb_state");
    if (!climb)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[Roll] Missing required port 'climb_state': %s",
                     climb.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    int climb_state = climb.value();

    if      (climb_state == static_cast<int>(ClimbState::DOWN)) rolling_up_ = true;
    else if (climb_state == static_cast<int>(ClimbState::UP))   rolling_up_ = false;
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "[Roll] Unknown climb_state");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Roll::onRunning()
{
    // First tick: fire the service request
    if (!request_sent_)
    {
        float roll_up_angle   = 0.0f;
        float roll_down_angle = 0.0f;

        try 
        {
            double up_temp = 0.0;
            double down_temp = 0.0;
            
            node_->get_parameter_or("roll.roll_up_angle", up_temp, 0.0);
            node_->get_parameter_or("roll.roll_down_angle", down_temp, 0.0);

            roll_up_angle = static_cast<float>(up_temp);
            roll_down_angle = static_cast<float>(down_temp);
        }
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), 
                         "[Roll] Exception caught while fetching parameters: %s. Failsafe activated.", 
                         e.what());
            return BT::NodeStatus::FAILURE;
        }

        if (!roll_up_client_->wait_for_service(500ms))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Roll] Service 'ak_motor_roll_up' not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request   = std::make_shared<motor_interfaces::srv::MotorState::Request>();
        request->data  = true;
        request->state = rolling_up_ ? roll_up_angle : roll_down_angle;

        RCLCPP_INFO(node_->get_logger(),
                    "[Roll] Sending command: ak_motor_roll_up  angle=%.2f  (rolling_up=%s)",
                    request->state, rolling_up_ ? "true" : "false");

        roll_up_client_->async_send_request(
            request,
            [this](rclcpp::Client<motor_interfaces::srv::MotorState>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                    RCLCPP_INFO(node_->get_logger(),
                                "[Roll] Service call succeeded: %s",
                                response->message.c_str());
                else
                    RCLCPP_WARN(node_->get_logger(),
                                "[Roll] Service call failed: %s",
                                response->message.c_str());

                std::lock_guard<std::mutex> lock(result_mutex_);
                call_succeeded_    = response->success;
                response_received_ = true;
            });

        request_sent_ = true;
        return BT::NodeStatus::RUNNING;
    }

    if (rolling_up_)
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (!response_received_)
            return BT::NodeStatus::RUNNING;

        RCLCPP_INFO(node_->get_logger(), "[Roll] roll_up complete");
        return call_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    if (rear_edge_)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[Roll] Rear edge detected — stopping roll_down");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void Roll::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[Roll] Halted");

    std::lock_guard<std::mutex> lock(result_mutex_);
    response_received_ = false;
    call_succeeded_    = false;
    request_sent_      = false;
}

void Roll::reset()
{
    rolling_up_        = false;
    response_received_ = false;
    call_succeeded_    = false;
    request_sent_      = false; 
}

} 