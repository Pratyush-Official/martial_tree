#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

struct StageLocation {
  float x;
  float y;
};

class GetLocationForStage : public BT::SyncActionNode
{
public:
  GetLocationForStage(const std::string & name,
                      const BT::NodeConfig & config,
                      rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  const std::unordered_map<int, StageLocation> location_map_ = {
    {4, {1.0f, 0.0f}},   // roi1_martial
    {5, {2.0f, 0.0f}},   // roi2_martial
    {6, {3.0f, 0.0f}},   // roi3_martial  
    {7, {4.0f, 0.0f}},   // roi4_martial
    {8, {5.0f, 0.0f}},   // roi5_martial
    {9, {6.0f, 0.0f}},   // roi6_martial
  };
};