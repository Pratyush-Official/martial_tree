#include "camera_nodes/get_roi_detections.hpp"
using namespace std::chrono;

GetROIDetections::GetROIDetections(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  client_ = node_->create_client<GetDetections>("/get_detections");
  RCLCPP_INFO(node_->get_logger(), "GetROIDetections initialized");
}

BT::PortsList GetROIDetections::providedPorts()
{
  return {
    BT::InputPort<int>("stage", 1, "Detection stage (1-4, or 4+ for martial ROI mode)"),
    BT::InputPort<int>("reset_delay_ms", 1200, "Delay after reset in ms"),
    BT::InputPort<int>("type", 2, "For stage>4: 1=check specific martial ROI, 2=standard logic"),
    BT::InputPort<std::string>("color", "red", "red=lower ROI, blue=higher ROI"), 
    BT::OutputPort<StageDetections>("detections")
  };
}

BT::NodeStatus GetROIDetections::onStart()
{
  getInput("stage", stage_);
  getInput("reset_delay_ms", reset_delay_ms_);
  getInput("type", type_);
  getInput("color", color_);  // added

  if (!client_->service_is_ready())
  {
    RCLCPP_WARN(node_->get_logger(), "Waiting for /get_detections service...");
    return BT::NodeStatus::FAILURE;
  }

  auto reset_req = std::make_shared<GetDetections::Request>();
  reset_req->reset = true;
  reset_req->stage = stage_;
  client_->async_send_request(reset_req);

  reset_time_ = steady_clock::now();
  phase_ = Phase::WAIT_AFTER_RESET;

  RCLCPP_INFO(node_->get_logger(),
    "BT: Reset sent for stage=%d, type=%d, color=%s",
    stage_, type_, color_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetROIDetections::onRunning()
{
  auto now = steady_clock::now();

  switch (phase_)
  {
    case Phase::WAIT_AFTER_RESET:
    {
      auto elapsed = duration_cast<milliseconds>(now - reset_time_).count();
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
      if (future_.wait_for(milliseconds(0)) != std::future_status::ready)
        return BT::NodeStatus::RUNNING;

      auto res = future_.get();

      StageDetections detections;
      detections.roi1         = res->roi1_class;
      detections.roi2         = res->roi2_class;
      detections.roi3         = res->roi3_class;
      detections.roi1_martial = res->roi1_martial;
      detections.roi2_martial = res->roi2_martial;
      detections.roi3_martial = res->roi3_martial;
      detections.roi4_martial = res->roi4_martial;
      detections.roi5_martial = res->roi5_martial;
      detections.roi6_martial = res->roi6_martial;
      detections.status       = res->status;
      setOutput("detections", detections);

      RCLCPP_INFO(
        node_->get_logger(),
        "BT Tick [stage %d, type %d, color %s]: "
        "roi1='%s', roi2='%s', roi3='%s', "
        "roi1_martial='%s', roi2_martial='%s', roi3_martial='%s', "
        "roi4_martial='%s', roi5_martial='%s', roi6_martial='%s', "
        "status='%s'",
        stage_, type_, color_.c_str(),
        detections.roi1.c_str(),
        detections.roi2.c_str(),
        detections.roi3.c_str(),
        detections.roi1_martial.c_str(),
        detections.roi2_martial.c_str(),
        detections.roi3_martial.c_str(),
        detections.roi4_martial.c_str(),
        detections.roi5_martial.c_str(),
        detections.roi6_martial.c_str(),
        detections.status.c_str());

      if (stage_ > 4 && type_ == 1)
      {
        const std::vector<std::string> martial_rois = {
          res->roi1_martial,
          res->roi2_martial,
          res->roi3_martial,
          res->roi4_martial,
          res->roi5_martial,
          res->roi6_martial
        };

        int roi_index = stage_ - 5;  // stage 5 -> index 0, stage 6 -> index 1, ...

        if (roi_index < static_cast<int>(martial_rois.size()) &&
            martial_rois[roi_index] == "yes")
        {
          RCLCPP_INFO(node_->get_logger(),
            "BT: SUCCESS — roi%d_martial is 'yes'", roi_index + 1);
          return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
      }

      // stage 1/2/3/4, or stage 5+ with type 2 — original logic
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
  phase_  = Phase::RESET;
  RCLCPP_WARN(node_->get_logger(), "BT: GetROIDetections halted");
}