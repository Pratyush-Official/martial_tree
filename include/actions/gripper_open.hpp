#ifndef GRIPPER_OPEN_HPP
#define GRIPPER_OPEN_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"


using namespace BT;

class GripperOpen : public BT::SyncActionNode
{
    public:
    GripperOpen(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};
#endif