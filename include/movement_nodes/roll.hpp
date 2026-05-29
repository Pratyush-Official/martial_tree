#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include "tree_flow_nodes/decision_nodes/decision_node.hpp"
#include <mutex>

namespace BT
{
class Roll : public BT::StatefulActionNode
{
public:
    Roll(const std::string &     name,
         const BT::NodeConfig &  config,
         rclcpp::Node::SharedPtr node);

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr roll_up_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr roll_down_client_;
    // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr roll_stop_client_;

    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr prox_front_sub_;
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr prox_rear_sub_;

    bool front_edge_ {false};
    bool rear_edge_  {false};

    bool rolling_up_       {false};
    bool response_received_{false};
    bool call_succeeded_   {false};

   

    std::mutex result_mutex_;
    std::mutex preset_mutex_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr selectClient() const;
   
    void reset();
};

} 