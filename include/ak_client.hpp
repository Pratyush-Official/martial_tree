#ifndef AK_CLIENT_HPP
#define AK_CLIENT_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/byte.hpp"

class AkClient : public BT::StatefulActionNode
{
public:
    AkClient(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus onRunning() override;
    BT::NodeStatus onStart() override;
    void onHalted() override;

private:

    bool request_sent;
    uint8_t prox_data;
    std_msgs::msg::Byte rear;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_up;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_down;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr active_client;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;
    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr rear_pub;

    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr prox_sub;
    
    void prox_callback(const std_msgs::msg::Byte::SharedPtr msg);
};

#endif
