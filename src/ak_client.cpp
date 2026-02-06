#include "ak_client.hpp"

AkClient::AkClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    client_up = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_up_execute");
    client_down = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_down_exec");

    rear_pub = node_->create_publisher<std_msgs::msg::Byte>("/rear_drive", 10);
    prox_sub = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_data", 10, std::bind(&AkClient::prox_callback, this, std::placeholders::_1));
    
    request_sent = false;
    rear.data = 0x00;
    prox_data = 0x00;
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
    active_client.reset();
    request_sent = false;
    future_ = std::future<std::shared_ptr<std_srvs::srv::SetBool::Response>>();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AkClient::onRunning()
{
    
    auto up_res = getInput<bool>("up");
    auto down_res = getInput<bool>("down");
    bool up = up_res.value_or(false);
    bool down = down_res.value_or(false);


    if (!active_client)
    {
        // Only trigger if we are in the starting state (0x09)
        if (prox_data == 0x09)
        {
            if (up && !down)
                active_client = client_up;
            else if (down && !up)
                active_client = client_down;
        }

        if (!active_client)
            return BT::NodeStatus::RUNNING;
    }

    if (!active_client->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_WARN(node_->get_logger(), "Service not available...");
        return BT::NodeStatus::RUNNING;
    }

    if (!request_sent)
    {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        future_ = active_client->async_send_request(req).future; 
        request_sent = true;
        return BT::NodeStatus::RUNNING;
    }

    if (future_.valid() && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto res = future_.get();
        if (!res->success)
            return BT::NodeStatus::FAILURE;
    }

    if (prox_data == 0x01)
    {
        rear.data = 0x69;
        rear_pub->publish(rear);
        return BT::NodeStatus::RUNNING;
    }

    if (prox_data == 0x08 || (request_sent && prox_data == 0x09))
    {
        rear.data = 0x00;
        rear_pub->publish(rear);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}


void AkClient::prox_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    prox_data = msg->data;
    printf("Prox data received: %d\n", prox_data);

}

void AkClient::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "AkClient halted!");
}
