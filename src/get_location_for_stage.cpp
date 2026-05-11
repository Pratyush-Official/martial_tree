#include "get_location_for_stage.hpp"

GetLocationForStage::GetLocationForStage(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::PortsList GetLocationForStage::providedPorts()
{
  return {
    BT::InputPort<int>("destination_stage"),
    BT::OutputPort<float>("setpoint_odom_1"),   // x
    BT::OutputPort<float>("setpoint_odom_2")    // y
  };
}

BT::NodeStatus GetLocationForStage::tick()
{
  int stage;
  getInput("destination_stage", stage);

  auto it = location_map_.find(stage);
  if (it == location_map_.end())
  {
    RCLCPP_ERROR(node_->get_logger(),
      "GetLocationForStage: no location for stage %d", stage);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("setpoint_odom_1", it->second.x);
  setOutput("setpoint_odom_2", it->second.y);

  RCLCPP_INFO(node_->get_logger(),
    "GetLocationForStage: stage=%d -> (%.2f, %.2f)",
    stage, it->second.x, it->second.y);

  return BT::NodeStatus::SUCCESS;
}