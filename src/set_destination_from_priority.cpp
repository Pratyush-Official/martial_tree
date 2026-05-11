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
    BT::InputPort<std::string>("color"),       // "red" or "blue"
    BT::OutputPort<int>("destination_stage")
  };
}

BT::NodeStatus SetDestinationFromPriority::tick()
{
  StageDetections det;
  std::string color;
  getInput("detections", det);
  getInput("color", color);

  // Priority tiers: {lower_roi_stage, higher_roi_stage}
  // stage 4=roi1, 5=roi2, 6=roi3, 7=roi4, 8=roi5, 9=roi6
  // Tier1: roi3(stage6), roi4(stage7)
  // Tier2: roi2(stage5), roi5(stage8)
  // Tier3: roi1(stage4), roi6(stage9)

  struct Tier {
    int low_stage;   // color=red picks this
    int high_stage;  // color=blue picks this
    std::string low_val;
    std::string high_val;
  };

  std::vector<Tier> priority_table = {
    {6, 7, det.roi3_martial, det.roi4_martial},  // tier 1
    {5, 8, det.roi2_martial, det.roi5_martial},  // tier 2
    {4, 9, det.roi1_martial, det.roi6_martial},  // tier 3
  };

  bool red = (color == "red");

  for (auto & tier : priority_table)
  {
    bool low_yes  = (tier.low_val  == "yes");
    bool high_yes = (tier.high_val == "yes");

    if (!low_yes && !high_yes) continue;  // nothing in this tier

    int destination = -1;

    if (low_yes && high_yes)
    {
      // both yes — color decides
      destination = red ? tier.low_stage : tier.high_stage;
    }
    else if (low_yes)
    {
      destination = tier.low_stage;
    }
    else
    {
      destination = tier.high_stage;
    }

    setOutput("destination_stage", destination);
    RCLCPP_INFO(node_->get_logger(),
      "SetDestination: destination_stage=%d (color=%s)",
      destination, color.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // all "no"
  RCLCPP_WARN(node_->get_logger(),
    "SetDestination: all ROIs are 'no' — returning FAILURE");
  return BT::NodeStatus::FAILURE;
}