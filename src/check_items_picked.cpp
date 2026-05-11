#include "check_items_picked.hpp"

CheckItemsPicked::CheckItemsPicked(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "CheckItemsPicked initialized");
}

BT::PortsList CheckItemsPicked::providedPorts()
{
  return {
    BT::InputPort<int>("items_picked", 0, "Number of items picked so far")
  };
}

BT::NodeStatus CheckItemsPicked::tick()
{
  int items;
  getInput("items_picked", items);

  if (items >= 2) {
    RCLCPP_INFO(node_->get_logger(),
      "CheckItemsPicked: items=%d >= 2, mission complete", items);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(),
    "CheckItemsPicked: items=%d, continue looping", items);
  return BT::NodeStatus::FAILURE;
}