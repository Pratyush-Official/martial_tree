#ifndef AK_UP_DOWN_HPP
#define AK_UP_DOWN_HPP

#include <string>
#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "/home/maomao/ROBOCON_26/camera/ws/src/motor_interfaces/srv/motor_state.hpp"

using InitClimbSrv = ar_interfaces::srv::InitClimb;

class InitClimb : public BT::StatefulActionNode
{
public:
    InitClimb(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<InitClimbSrv>::SharedPtr client_;
    std::shared_future<InitClimbSrv::Response::SharedPtr> future_;
};

#endif