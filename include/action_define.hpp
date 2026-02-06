#ifndef ACTION_DEFINE_HPP
#define ACTION_DEFINE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using namespace BT;

enum ActionType
{
    NAVIGATE_FROM_START_ZONE,
    NAVIGATE_FROM_RETRY_ZONE,
    ALIGN_WITH_RACK,
    ROTATE_TO_FACE_MR,
    ROTATE_TO_FACE_RACK,
};

class ActionDefine : public BT::SyncActionNode
{
    public:
    ActionDefine(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};