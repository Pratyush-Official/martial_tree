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
    BT::InputPort<int>("stage", 1, "Detection stage"),
    BT::InputPort<int>("reset_delay_ms", 1200, "Delay after reset in ms"),
    BT::InputPort<int>("type", 2, "1=check specific martial ROI, 2=standard logic"),
    BT::InputPort<std::string>("color", "red", "red=lower ROI, blue=higher ROI"),
    BT::OutputPort<StageDetections>("detections")
  };
}

BT::NodeStatus GetROIDetections::onStart()
{
  getInput("stage", stage_);
  getInput("reset_delay_ms", reset_delay_ms_);
  getInput("type", type_);
  getInput("color", color_);

  if (!client_->service_is_ready())
  {
    RCLCPP_WARN(node_->get_logger(),
      "Service not ready on start, waiting... [stage=%d type=%d]",
      stage_, type_);
    phase_ = Phase::WAIT_SERVICE;
    return BT::NodeStatus::RUNNING;
  }

  sendReset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetROIDetections::onRunning()
{
  auto now = steady_clock::now();

  switch (phase_)
  {
    case Phase::WAIT_SERVICE:
    {
      if (!client_->service_is_ready())
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(),
          *node_->get_clock(), 2000,
          "Still waiting for /get_detections service... [stage=%d type=%d]",
          stage_, type_);
        return BT::NodeStatus::RUNNING;
      }
      // service appeared
      RCLCPP_INFO(node_->get_logger(),
        "Service detected, proceeding [stage=%d type=%d]",
        stage_, type_);
      sendReset();
      return BT::NodeStatus::RUNNING;
    }

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
      detections.stage11_result = res->stage11_result;
      detections.stage12_roi1 = res->stage12_roi1;
      detections.stage12_roi2 = res->stage12_roi2;
      detections.stage12_roi3 = res->stage12_roi3;
      detections.status       = res->status;
      setOutput("detections", detections);

      RCLCPP_INFO(
        node_->get_logger(),
        "BT Tick [stage %d, type %d, color %s]: "
        "roi1='%s', roi2='%s', roi3='%s', "
        "roi1_martial='%s', roi2_martial='%s', roi3_martial='%s', "
        "roi4_martial='%s', roi5_martial='%s', roi6_martial='%s', "
        "stage11_result='%s', stage12_roi1='%s', stage12_roi2='%s', stage12_roi3='%s',"
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
        deletections.stage11_result.c_str(),
        detections.stage12_roi1.c_str(),
        detections.stage12_roi2.c_str(),
        detections.stage12_roi3.c_str(),
        detections.status.c_str());

      if (stage_ > 4 && stage_ <= 10 && type_ == 1)
      {
        const std::vector<std::string> martial_rois = {
          res->roi1_martial,
          res->roi2_martial,
          res->roi3_martial,
          res->roi4_martial,
          res->roi5_martial,
          res->roi6_martial
        };

        int roi_index = stage_ - 5;

        if (roi_index < static_cast<int>(martial_rois.size()) &&
            martial_rois[roi_index] == "yes")
        {
          RCLCPP_INFO(node_->get_logger(),
            "BT: SUCCESS — roi%d_martial is 'yes'", roi_index + 1);
          return BT::NodeStatus::SUCCESS;
        }

        // service was available, result is genuinely not "yes"
        RCLCPP_INFO(node_->get_logger(),
          "BT: FAILURE — roi%d_martial is not 'yes' (genuine no)",
          roi_index + 1);
        return BT::NodeStatus::FAILURE;
      }

      if (stage_ == 11)
      {
        const std::vector<std::string> stage11_res = {
          res->stage11_result,

        };
        if (stage11_res[0] == "yes")
        {
          RCLCPP_INFO(node_->get_logger(),
            "BT: SUCCESS — stage11_result is 'yes'");
          return BT::NodeStatus::SUCCESS;
        }

        // service was available, result is genuinely not "yes"
        RCLCPP_INFO(node_->get_logger(),
          "BT: FAILURE — stage11_result is not 'yes' (genuine no)");
        return BT::NodeStatus::FAILURE;
      }

      // type 2 or stage outside 5 to 10— original logic
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

void GetROIDetections::sendReset()
{
  auto reset_req = std::make_shared<GetDetections::Request>();
  reset_req->reset = true;
  reset_req->stage = stage_;
  client_->async_send_request(reset_req);
  reset_time_ = steady_clock::now();
  phase_ = Phase::WAIT_AFTER_RESET;
  RCLCPP_INFO(node_->get_logger(),
    "BT: Reset sent for stage=%d, type=%d, color=%s",
    stage_, type_, color_.c_str());
}