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
    roll_up_client_   = node_->create_client<std_srvs::srv::SetBool>("ak_motor_roll_up");
    roll_down_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_roll_down");
    // roll_stop_client_ = node_->create_client<std_srvs::srv::SetBool>("ak_motor_roll_stop");

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
        BT::InputPort<int>("climb_state",
            "Climb state: UP or DOWN"),
       
        
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

   

    auto client = selectClient();

    if (!client->wait_for_service(500ms))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[Roll] Service '%s' not available",
                     rolling_up_ ? "roll_up_exec" : "roll_down_exec");
        return BT::NodeStatus::FAILURE;
    }

    auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    RCLCPP_INFO(node_->get_logger(),
                "[Roll] Sending main command: %s",
                rolling_up_ ? "roll_up_exec" : "roll_down_exec");

    client->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
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

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Roll::onRunning()
{
   
    bool prox_stop = false;

    if (rolling_up_ && !front_edge_)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[Roll] Front edge cleared  stopping roll_up");
        prox_stop = true;
    }
    else if (!rolling_up_ && rear_edge_)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[Roll] Rear edge detected stopping roll_down");
        prox_stop = true;
    }

    if (prox_stop)
    {
        // if (roll_stop_client_->service_is_ready())
        // {
        //     auto req  = std::make_shared<std_srvs::srv::SetBool::Request>();
        //     req->data = true;
        //     roll_stop_client_->async_send_request(req);
        // }
        // else
        // {
        //     RCLCPP_WARN(node_->get_logger(),
        //                 "[Roll] roll_stop_exec not available skipping stop call");
        
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;

    // std::lock_guard<std::mutex> lock(result_mutex_);
    // if (!response_received_)
    //     return BT::NodeStatus::RUNNING;

    // return call_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void Roll::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "[Roll] Halted");

    std::lock_guard<std::mutex> lock(result_mutex_);
    response_received_ = false;
    call_succeeded_    = false;
}

rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr Roll::selectClient() const
{
    return rolling_up_ ? roll_up_client_ : roll_down_client_;
}



void Roll::reset()
{
    rolling_up_        = false;
    response_received_ = false;
    call_succeeded_    = false;
}

} 