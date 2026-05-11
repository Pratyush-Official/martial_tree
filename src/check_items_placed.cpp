BT::NodeStatus CheckItemsPicked::tick()
{
  int items;
  getInput("items_picked", items);
  if (items >= 2) {
    RCLCPP_INFO(node_->get_logger(), "Items picked = 2, mission complete");
    return BT::NodeStatus::SUCCESS; 
  }
  return BT::NodeStatus::FAILURE;  
}