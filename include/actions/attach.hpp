#ifndef ATTACH_HPP
#define ATTACH_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"


using namespace BT;

class Attach : public BT::SyncActionNode
{
    public:
    Attach(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};
#endif