// #include "ak_client.hpp"

// AkClient::AkClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
//     : BT::StatefulActionNode(name, config), node_(node)
// {
//     client_up = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_up_execute");
//     client_down = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_down_exec");

//     rear_pub = node_->create_publisher<std_msgs::msg::Byte>("/rear_drive", 10);
//     prox_sub = node_->create_subscription<std_msgs::msg::Byte>(
//         "/prox_data", 10, std::bind(&AkClient::prox_callback, this, std::placeholders::_1));
    
//     request_sent = false;
//     rear.data = 0x00;
//     prox_data = 0x00;
// }

// BT::PortsList AkClient::providedPorts()
// {
//     return {
//         BT::InputPort<bool>("up", "Send true to raise the mechanism"),
//         BT::InputPort<bool>("down", "Send true to lower the mechanism")
//     };
// }

// BT::NodeStatus AkClient::onStart()
// {
//     active_client.reset();
//     request_sent = false;

//     service_done_ = false;
//     service_success_ = false;

//     return BT::NodeStatus::RUNNING;
// }

// BT::NodeStatus AkClient::onRunning()
// {
    
//     auto up_res = getInput<bool>("up");
//     auto down_res = getInput<bool>("down");
//     bool up = up_res.value_or(false);
//     bool down = down_res.value_or(false);


//     if (!active_client)
//     {
//         // Only trigger if we are in the starting state (0x09)
//         if (prox_data == 0x09)
//         {
//             if (up && !down)
//                 active_client = client_up;
//             else if (down && !up)
//                 active_client = client_down;
//         }

//         if (!active_client)
//             return BT::NodeStatus::RUNNING;
//     }

//     if (!active_client->wait_for_service(std::chrono::milliseconds(100)))
//     {
//         RCLCPP_WARN(node_->get_logger(), "Service not available...");
//         return BT::NodeStatus::RUNNING;
//     }

//     if (!request_sent)
//     {
//         auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
//         req->data = true;
//         future_ = active_client->async_send_request(req).future; 
//         request_sent = true;
//         return BT::NodeStatus::RUNNING;
//     }

//     // if (!service_done_)
//     // {
//     //     if (future_.valid() &&
//     //         future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
//     //     {
//     //         auto res = future_.get();
//     //         service_done_ = true;
//     //         service_success_ = res->success;
//     //         RCLCPP_INFO(node_->get_logger(), "Service call completed with response: %s", res->success ? "true" : "false");

//     //         if (!service_success_){
//     //             return BT::NodeStatus::FAILURE;
//     //     }
//     //     }
//     // }

//     if (request_sent && future_.valid())
//     {
//         auto status = future_.wait_for(std::chrono::seconds(0));
//         if (status == std::future_status::ready)
//         {
//             auto res = future_.get();
//             service_done_ = true;
//             service_success_ = res->success;
//             RCLCPP_INFO(node_->get_logger(), "Service done: %s", res->success ? "true" : "false");
//         }
//     }


//     if (prox_data == 0x01)
//     {
//         rear.data = 0x69;
//         printf("Sending rear data: %d\n", rear.data);
//         rear_pub->publish(rear);
//         return BT::NodeStatus::RUNNING;
//     }   
//     if (prox_data == 0x08 || (request_sent && prox_data == 0x09))
//     {
//         rear.data = 0x00;
//         rear_pub->publish(rear);
//         return BT::NodeStatus::SUCCESS;
//     }   
//     return BT::NodeStatus::RUNNING;
//     }


// void AkClient::prox_callback(const std_msgs::msg::Byte::SharedPtr msg)
// {
//     prox_data = msg->data;
//     // printf("Prox data received: %d\n", prox_data);

// }

// void AkClient::onHalted()
// {
//     RCLCPP_WARN(node_->get_logger(), "AkClient halted!");
// }

#include "ak_client.hpp"

AkClient::AkClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    client_up = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_up_execute");
    client_down = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_down_exec");

    rear_pub = node_->create_publisher<std_msgs::msg::Byte>("/rear_drive", 10);

    prox_sub = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_data", 10, std::bind(&AkClient::prox_callback, this, std::placeholders::_1));

    // Initialize variables
    prox_data = 0x00;
    rear.data = 0x00;
    request_sent = false;
    service_done_ = false;
    service_success_ = false;
    active_client = nullptr;
}

BT::PortsList AkClient::providedPorts()
{
    return {
        BT::InputPort<bool>("up", "Send true to raise the mechanism"),
        BT::InputPort<bool>("down", "Send true to lower the mechanism")
    };
}

BT::NodeStatus AkClient::onStart()
{
    // Reset state for a new tick
    request_sent = false;
    service_done_ = false;
    service_success_ = false;
    active_client = nullptr;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AkClient::onRunning()
{
    bool up = getInput<bool>("up").value_or(false);
    bool down = getInput<bool>("down").value_or(false);

    // Choose service based on prox_data and input
    if (!active_client && prox_data == 0x09)
    {
        if (up && !down)
            active_client = client_up;
        else if (down && !up)
            active_client = client_down;
    }

    // Still no valid client? keep running
    if (!active_client)
        return BT::NodeStatus::RUNNING;

    // Wait for service to be available
    if (!active_client->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_WARN(node_->get_logger(), "Service not available yet...");
        return BT::NodeStatus::RUNNING;
    }

    // Send request once
    if (!request_sent)
    {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        future_ = active_client->async_send_request(req).future;
        request_sent = true;
        return BT::NodeStatus::RUNNING;
    }

    // Check if service response is ready
    if (request_sent && future_.valid() && !service_done_)
    {
        auto status = future_.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready)
        {
            auto res = future_.get();
            service_done_ = true;
            service_success_ = res->success;
            RCLCPP_INFO(node_->get_logger(),
                        "Service completed: %s",
                        service_success_ ? "true" : "false");
        }
    }

    // Act based on prox_data
    switch (prox_data)
    {
        case 0x01:
            rear.data = 0x69;
            rear_pub->publish(rear);
            return BT::NodeStatus::RUNNING;

        case 0x08:
        case 0x09:
            rear.data = 0x00;
            rear_pub->publish(rear);
            return BT::NodeStatus::SUCCESS;

        default:
            return BT::NodeStatus::RUNNING;
    }
}

void AkClient::prox_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    prox_data = msg->data;
}

void AkClient::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "AkClient halted!");
    // Optionally reset rear motor
    rear.data = 0x00;
    rear_pub->publish(rear);
}

