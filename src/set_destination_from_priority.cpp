#include "set_destination_from_priority.hpp"

SetDestinationFromPriority::SetDestinationFromPriority(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::PortsList SetDestinationFromPriority::providedPorts()
{
  return {
    BT::InputPort<StageDetections>("detections"),
    BT::InputPort<std::string>("color"),
    BT::OutputPort<int>("destination_stage"),
    BT::OutputPort<std::string>("detection_status")
    // detection_status values:
    // "found"   — at least one "yes", destination updated
    // "outside" — at least one "outside", zero "yes"
    // "none"    — all "no", tree should stop
  };
}

BT::NodeStatus SetDestinationFromPriority::tick()
{
  StageDetections det;
  std::string color;
  getInput("detections", det);
  getInput("color", color);

  // collect all 6 martial ROI values with their stage numbers
  // stage 4=roi1, 5=roi2, 6=roi3, 7=roi4, 8=roi5, 9=roi6
  struct ROIEntry {
    std::string value;
    int stage;
  };

  // priority tiers: {low_roi(red), high_roi(blue)}
  // tier 1: roi3(6), roi4(7)
  // tier 2: roi2(5), roi5(8)
  // tier 3: roi1(4), roi6(9)
  struct Tier {
    ROIEntry low;   // color=red prefers this
    ROIEntry high;  // color=blue prefers this
  };

  std::vector<Tier> priority_table = {
    { {det.roi3_martial, 7}, {det.roi4_martial, 8} },  // tier 1
    { {det.roi2_martial, 6}, {det.roi5_martial, 9} },  // tier 2
    { {det.roi1_martial, 5}, {det.roi6_martial, 10} },  // tier 3
  };

  bool red = (color == "red");

  // first pass: look for "yes"
  for (auto & tier : priority_table)
  {
    bool low_yes  = (tier.low.value  == "yes");
    bool high_yes = (tier.high.value == "yes");

    if (!low_yes && !high_yes) continue;

    int destination = -1;
    if (low_yes && high_yes)
      destination = red ? tier.low.stage : tier.high.stage;
    else if (low_yes)
      destination = tier.low.stage;
    else
      destination = tier.high.stage;

    setOutput("destination_stage", destination);
    setOutput("detection_status", std::string("found"));
    RCLCPP_INFO(node_->get_logger(),
      "SetDestination: found — destination_stage=%d (color=%s)",
      destination, color.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // second pass: check for "outside" (no "yes" found above)
  bool any_outside = false;
  for (auto & tier : priority_table)
  {
    if (tier.low.value  == "outside" ||
        tier.high.value == "outside")
    {
      any_outside = true;
      break;
    }
  }

  if (any_outside)
  {
    setOutput("detection_status", std::string("outside"));
    // destination_stage will be overridden to 5 by the tree
    RCLCPP_WARN(node_->get_logger(),
      "SetDestination: outside detections found — recovery to stage 5");
    return BT::NodeStatus::SUCCESS;
  }

  // all "no"
  setOutput("detection_status", std::string("none"));
  RCLCPP_WARN(node_->get_logger(),
    "SetDestination: all none — tree should stop");
  return BT::NodeStatus::FAILURE;
}