#ifndef GRIPPER_CLOSE_HPP
#define GRIPPER_CLOSE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"


using namespace BT;

class GripperClose : public BT::SyncActionNode
{
    public:
    GripperClose(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};
#endif