#include "get_roi_detections.hpp"

using namespace std::chrono;

GetROIDetections::GetROIDetections(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  client_ = node_->create_client<GetDetections>("/get_detections");
}

BT::PortsList GetROIDetections::providedPorts()
{
  return {
    BT::InputPort<int>("stage", 1, "Detection stage (1-3)"),
    BT::InputPort<int>("reset_delay_ms", 1200, "Delay after reset in ms"),
    BT::OutputPort<StageDetections>("detections")
  };
}

BT::NodeStatus GetROIDetections::onStart()
{
  getInput("stage", stage_);
  getInput("reset_delay_ms", reset_delay_ms_);

  if (!client_->service_is_ready())
  {
    RCLCPP_WARN(node_->get_logger(),
                "Waiting for /get_detections service...");
    return BT::NodeStatus::FAILURE;
  }

  auto reset_req = std::make_shared<GetDetections::Request>();
  reset_req->reset = true;
  reset_req->stage = stage_;

  client_->async_send_request(reset_req);

  reset_time_ = steady_clock::now();
  phase_ = Phase::WAIT_AFTER_RESET;

  RCLCPP_INFO(node_->get_logger(),
              "BT: Reset sent for stage=%d", stage_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetROIDetections::onRunning()
{
  auto now = steady_clock::now();

  switch (phase_)
  {
    case Phase::WAIT_AFTER_RESET:
    {
      auto elapsed =
        duration_cast<milliseconds>(now - reset_time_).count();

      if (elapsed < reset_delay_ms_)
        return BT::NodeStatus::RUNNING;

      phase_ = Phase::REQUEST;
      return BT::NodeStatus::RUNNING;
    }

    case Phase::REQUEST:
    {
      auto req = std::make_shared<GetDetections::Request>();
      req->reset = false;
      req->stage = stage_;

      future_ = client_->async_send_request(req).share();
      phase_ = Phase::WAIT_RESPONSE;

      RCLCPP_INFO(node_->get_logger(), "BT: Detection request sent");
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_RESPONSE:
    {
      if (future_.wait_for(milliseconds(0)) !=
          std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }

      auto res = future_.get();

      StageDetections detections;
      detections.roi1   = res->roi1_class;
      detections.roi2   = res->roi2_class;
      detections.roi3   = res->roi3_class;
      detections.status = res->status;

      setOutput("detections", detections);

      RCLCPP_INFO(
        node_->get_logger(),
        "BT Tick [stage %d]: roi1='%s', roi2='%s', roi3='%s', status='%s'",
        stage_,
        detections.roi1.c_str(),
        detections.roi2.c_str(),
        detections.roi3.c_str(),
        detections.status.c_str());

      if (res->success)
      {
        RCLCPP_INFO(node_->get_logger(), "BT: SUCCESS");
        return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::RUNNING;
    }

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void GetROIDetections::onHalted()
{
  future_ = rclcpp::Client<GetDetections>::SharedFuture();
  phase_ = Phase::RESET;

  RCLCPP_WARN(node_->get_logger(),
              "BT: GetROIDetections halted");
}
