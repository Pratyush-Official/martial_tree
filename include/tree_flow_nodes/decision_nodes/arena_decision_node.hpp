#pragma once

#include <string>
#include <memory>
#include "behaviortree_cpp/action_node.h"
#include "camera_nodes/get_roi_detections.hpp"   // for StageDetections

// forward-declare tree_ticker so we don't need to pull in the full header
class tree_ticker;

namespace bt_nodes
{

class ArenaDecisionNode : public BT::SyncActionNode
{
public:
  // Third argument matches the registration pattern used in bt_main.cpp
  // but is intentionally unused — ArenaDecisionNode needs no ROS node handle.
  ArenaDecisionNode(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<tree_ticker> /*unused*/ = nullptr);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace bt_nodes