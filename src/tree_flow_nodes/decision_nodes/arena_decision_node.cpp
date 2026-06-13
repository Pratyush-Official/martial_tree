#include "tree_flow_nodes/decision_nodes/arena_decision_node.hpp"

namespace bt_nodes
{

ArenaDecisionNode::ArenaDecisionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<tree_ticker> /*unused*/)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ArenaDecisionNode::providedPorts()
{
  return {
    BT::InputPort<StageDetections>("detections", "StageDetections struct from GetROIDetections"),
    BT::OutputPort<double>("move_x", "Target X movement"),
    BT::OutputPort<double>("move_y", "Target Y movement"),
    BT::OutputPort<std::string>("dxn", "Direction: L, R, or M"),
  };
}

BT::NodeStatus ArenaDecisionNode::tick()
{
  StageDetections detections;
  if (!getInput("detections", detections)) {
    return BT::NodeStatus::FAILURE;
  }

  const std::string & roi1 = detections.stage12_roi1;
  const std::string & roi2 = detections.stage12_roi2;
  const std::string & roi3 = detections.stage12_roi3;
  double move_x = 0.0;
  double move_y = 0.0;
  char dxn = 'F';

  if (roi1 == "yes") {
    // ROI 1 wins — highest priority (regardless of roi2/roi3)
    move_x = 1.0;   // TODO: tune
    move_y = 1.0;
    dxn = 'L';   // TODO: tune
  } else if (roi3 == "yes") {
    // ROI 3 — second priority (roi1 is not "yes")
    move_x = 2.0;   // TODO: tune
    move_y = 2.0; 
    dxn = 'R';  // TODO: tune
  } else if (roi2 == "yes") {
    // ROI 2 only — lowest priority (roi1 and roi3 are not "yes")
    move_x = 3.0;   // TODO: tune
    move_y = 3.0;
    dxn = 'M';   // TODO: tune
  } else {
    // No useful detection (no/blocked) — zero out
    move_x = 0.0;
    move_y = 0.0;
  }

  setOutput("move_x", move_x);
  setOutput("move_y", move_y);
  setOutput("dxn", dxn);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes