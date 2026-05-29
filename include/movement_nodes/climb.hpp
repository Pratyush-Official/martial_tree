#pragma once   
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tree_flow_nodes/decision_nodes/decision_node.hpp"
#include <memory>
#include <mutex>
#include <string>

class Climb : public BT::StatefulActionNode
{
public:
    Climb(const std::string & name,
          const BT::NodeConfig & config,
          rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ak_up_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ak_down_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ak_stop_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr roll_up_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr roll_down_client_;

    int  climb_state_       {0};

    std::mutex result_;
    bool response_received_ {false};
    bool call_succeeded_    {false};
    bool preset_sent_       {false};
    bool preset_succeded_   {false};

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr selectClient(int state);
    void sendPreset();
};