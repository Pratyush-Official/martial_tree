#ifndef GET_ROI_DETECTIONS_HPP_
#define GET_ROI_DETECTIONS_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "oakd_roi_detector_interfaces/srv/get_detections.hpp"
#include "qos_profiles.hpp"

struct StageDetections
{
  std::string roi1;
  std::string roi2;
  std::string roi3;
  std::string roi1_martial;
  std::string roi2_martial;
  std::string roi3_martial;
  std::string roi4_martial;
  std::string roi5_martial;
  std::string roi6_martial;
  std::string status;
};

class GetROIDetections : public BT::StatefulActionNode
{
public:
  using GetDetections =
    oakd_roi_detector_interfaces::srv::GetDetections;

  GetROIDetections(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    RESET,
    WAIT_AFTER_RESET,
    REQUEST,
    WAIT_RESPONSE
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<GetDetections>::SharedPtr client_;
  rclcpp::Client<GetDetections>::SharedFuture future_;

  Phase phase_{Phase::RESET};

  std::chrono::steady_clock::time_point reset_time_;
  std::string color_ = "red";
  int stage_{0};
  int type_ = 2;
  int reset_delay_ms_{1200};
};

#endif  // GET_ROI_DETECTIONS_HPP_
