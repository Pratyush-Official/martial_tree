#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class CheckItemsPicked : public BT::SyncActionNode
{
public:
  CheckItemsPicked(const std::string & name,
                   const BT::NodeConfig & config,
                   rclcpp::Node::SharedPtr node);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
};