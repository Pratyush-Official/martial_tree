#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <string>
#include <memory>

// Forward declare tree_ticker
class tree_ticker;

using BT::NodeStatus;

class SetpointLoader : public BT::SyncActionNode {
public:
  SetpointLoader(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  NodeStatus tick() override;

private:
  std::string yaml_path_;
  std::shared_ptr<tree_ticker> ticker_;
};