#ifndef FIND_HIDDEN_SCROLL_HPP_
#define FIND_HIDDEN_SCROLL_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <string>
#include <map>

#include "get_roi_detections.hpp" 
#include "qos_profiles.hpp"

class FindHiddenScroll : public BT::SyncActionNode
{
public:
  FindHiddenScroll(const std::string& name,
                   const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

#endif  // FIND_HIDDEN_SCROLL_HPP_
