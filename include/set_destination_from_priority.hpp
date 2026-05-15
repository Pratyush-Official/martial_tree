#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <camera_nodes/get_roi_detections.hpp>

class SetDestinationFromPriority : public BT::SyncActionNode
{
public:
  SetDestinationFromPriority(const std::string & name,
                              const BT::NodeConfig & config,
                              rclcpp::Node::SharedPtr node);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
};