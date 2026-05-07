#include "find_hidden_scroll.hpp"

FindHiddenScroll::FindHiddenScroll(
  const std::string& name,
  const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList FindHiddenScroll::providedPorts()
{
  return {
    BT::InputPort<StageDetections>("stage1"),
    BT::InputPort<StageDetections>("stage2"),
    BT::InputPort<StageDetections>("stage3"),
    BT::InputPort<StageDetections>("stage4"),
    // we overwrite stage4
    BT::OutputPort<StageDetections>("stage4")
  };
}

BT::NodeStatus FindHiddenScroll::tick()
{
  StageDetections s1, s2, s3, s4;

  if (!getInput("stage1", s1) ||
      !getInput("stage2", s2) ||
      !getInput("stage3", s3) ||
      !getInput("stage4", s4))
  {
    return BT::NodeStatus::FAILURE;
  }

  // Total expected scrolls
  std::map<std::string, int> remaining = {
    {"fake", 1},
    {"real", 4},
    {"mr",   3}
  };

  int detected = 0;

  auto consume = [&](const std::string& v) -> bool
  {
    if (v == "none")
      return true;

    if (remaining.count(v) == 0 || remaining[v] == 0)
      return false;

    remaining[v]--;
    detected++;
    return true;
  };

  // Stages 1–3 (all ROIs)
  for (const auto& v : {s1.roi1, s1.roi2, s1.roi3,
                        s2.roi1, s2.roi2, s2.roi3,
                        s3.roi1, s3.roi2, s3.roi3})
  {
    if (!consume(v))
      return BT::NodeStatus::FAILURE;
  }

  // Stage 4 — IGNORE roi2
  if (!consume(s4.roi1)) return BT::NodeStatus::FAILURE;
  if (!consume(s4.roi3)) return BT::NodeStatus::FAILURE;

  if (detected != 7)
    return BT::NodeStatus::FAILURE;

  // Find the missing one
  std::string hidden;
  for (const auto& [type, count] : remaining)
  {
    if (count == 1)
    {
      hidden = type;
      break;
    }
  }

  if (hidden.empty())
    return BT::NodeStatus::FAILURE;

  s4.roi2 = hidden;
  std::cout << "[FindHiddenScroll] Hidden scroll inferred: "
          << hidden << std::endl;


  setOutput("stage4", s4);

  return BT::NodeStatus::SUCCESS;
}
