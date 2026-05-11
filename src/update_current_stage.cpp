#include "update_current_stage.hpp"

UpdateCurrentStage::UpdateCurrentStage(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::PortsList UpdateCurrentStage::providedPorts()
{
  return {
    BT::InputPort<int>("destination_stage"),
    BT::OutputPort<int>("current_stage")
  };
}

BT::NodeStatus UpdateCurrentStage::tick()
{
  int dest;
  getInput("destination_stage", dest);
  setOutput("current_stage", dest);
  RCLCPP_INFO(node_->get_logger(),
    "UpdateCurrentStage: current_stage=%d", dest);
  return BT::NodeStatus::SUCCESS;
}