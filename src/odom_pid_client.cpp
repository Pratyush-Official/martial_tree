// #include "odom_client.hpp"
// #include "qos_profiles.hpp"
 
// using namespace std::placeholders;
// OdomPIDClient::OdomPIDClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : 
//     BT::StatefulActionNode(name, config), node_(node)
// {

//         odom_client = rclcpp_action::create_client<ar_interfaces::action::Movement>(node, "/movement_odom"); //movement_pid is the service name

//         RCLCPP_INFO(node_->get_logger(),"Odom Pid: initialized");
//         done_flag = false;
// }

// BT::PortsList OdomPIDClient::providedPorts()
// {
//     return {};
// }

// BT::NodeStatus OdomPIDClient::onStart()
// {
//     done_flag = false;
//     success = false;
//     auto goal_options = rclcpp_action::Client<ar_interfaces::action::Movement>::SendGoalOptions();
//     goal_options.goal_response_callback = std::bind(&OdomPIDClient::goal_response_callback, this, _1); //When server sends a response to a goal request, run node_ callback
//     goal_options.feedback_callback = std::bind(&OdomPIDClient::feedback_callback, this, _1, _2);
//     goal_options.result_callback = std::bind(&OdomPIDClient::result_callback, this, _1);

//     goal_msg.setpoint_odom = std::vector<float>{2.0f, 0.0f, 0.0f, 0.0f};
//     goal_msg.tolerance_odom = 0.0;
//     goal_msg.timeout_odom = 0.0;

//     start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
//                  std::chrono::steady_clock::now().time_since_epoch())
//                  .count();
    
//     if (!client_ptr_spear->wait_for_action_server(std::chrono::seconds(5))) {
//         RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
//         done_flag = true;
//         success = false;
//         return BT::NodeStatus::FAILURE;
//     }

//     client_ptr_spear->async_send_goal(goal_msg, goal_options);
//     RCLCPP_INFO(node_->get_logger(), "Odom::sent goal");
    
//     return BT::NodeStatus::RUNNING;
// }

// BT::NodeStatus OdomPIDClient::onRunning()
//     {
//     uint32_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
//                  std::chrono::steady_clock::now().time_since_epoch())
//                  .count();

//     // if ( now - start_time >= 3000)
//     //     {
//     //      if( fabs(prev_x - odom_msg.pose.pose.position.x) < 0.01 && fabs((prev_y - odom_msg.pose.pose.position.y)) < 0.01)
//     //      {
//     //          cancel_goal();
//     //          done_flag = true;
//     //          is_on_line = false;
//     //          RCLCPP_INFO(node_->get_logger(), " RecoveryNode::Robot static.. cancel goal ");
//     //          return BT::NodeStatus::SUCCESS;
//     //      }
//     //      else
//     //      {
//     //          start_time = get_tick_ms();
//     //      }
//     //      prev_x = odom_msg.pose.pose.position.x;
//     //      prev_y = odom_msg.pose.pose.position.y;
     
//     //  if ( fabs(odom_msg.pose.pose.position.y) >= 2.70 && fabs(odom_msg.pose.pose.position.x -  goal_msg.pose.pose.position.x) < 0.3 )
//     //  {
//     //      if (is_on_line)
//     //      {
//     //          cancel_goal();

//     //         done_flag = true;
//     //         is_on_line = false;
//     //         x_horiz_line_detected = false;

//     //      }
//     // }
//     if (this->done_flag && this->success)
//     {
//         return BT::NodeStatus::SUCCESS; 
//         RCLCPP_INFO(node_->get_logger(), "Success Sent");
//     }
    
//     if (this->done_flag && !this->success)
//     {
//         return BT::NodeStatus::FAILURE;
//     }

//     return BT::NodeStatus::RUNNING;
// }

// void OdomPIDClient::onHalted()
// {
//     cancel_goal();  
// }

// void OdomPIDClient::cancel_goal()
// {
//     if (goal_handle)
//     {
//         client_ptr_spear->async_cancel_goal(goal_handle);
//         RCLCPP_INFO(node_->get_logger(), "OdomPIDClient::Goal canceled");
//     }
// }

// void OdomPIDClient::goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_)
// {

//     if (!goal_handle_)
//     {
//         RCLCPP_ERROR(node_->get_logger(),
//                      "OdomPIDClient::Goal rejected by server");
//         this->done_flag = true;
//         this->success = false;
//     }
//     else
//     {
//         RCLCPP_INFO(node_->get_logger(),
//                     "OdomPIDClient::Goal accepted");
//         goal_handle = goal_handle_;
//     }
//     // if (!goal_handle_)
//     // {
//     //     RCLCPP_ERROR(node_->get_logger(), "OdomPIDClient::Navigate to spear %i goal rejected by server", node_->silo_numbers.data[0]);
//     //     setOutput<uint8_t>("Op_SiloNumber", 0);
//     // }
//     // else
//     // {
//     //     RCLCPP_INFO(node_->get_logger(), "OdomPIDClient::Navigating to spear %i ", node_->silo_numbers.data[0]);
//     //     node_->goal_handle = goal_handle_;

//     // }

// }

// void OdomPIDClient::result_callback(const GoalHandleMovement::WrappedResult &result)
// {
//     if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
//     {
//         RCLCPP_INFO(node_->get_logger(),
//                     "OdomPIDClient::Goal succeeded");
//         this->success = true;
//     }
//     else
//     {
//         RCLCPP_WARN(node_->get_logger(),
//                     "OdomPIDClient::Goal failed or canceled");
//         this->success = false;
//     }
//     this->done_flag = true;
// }

// void OdomPIDClient::feedback_callback(
//     GoalHandleMovement::SharedPtr,
//     const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback)
// {
//     (void)feedback;

// }
#include "odom_pid_client.hpp"

using namespace std::placeholders;

OdomPIDClient::OdomPIDClient(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
    odom_client_ =
        rclcpp_action::create_client<Movement>(node_, "/movement_odom");

    RCLCPP_INFO(node_->get_logger(), "OdomPIDClient initialized");
}

BT::PortsList OdomPIDClient::providedPorts()
{
    return {
        BT::InputPort<float>("setpoint_odom_1"),
        BT::InputPort<float>("setpoint_odom_2"),
        BT::InputPort<float>("tolerance"),
        BT::InputPort<float>("timeout")
    };
}

BT::NodeStatus OdomPIDClient::onStart()
{
    done_ = false;
    success_ = false;

    if (!odom_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node_->get_logger(), "Odom action server not available");
        return BT::NodeStatus::FAILURE;
    }

    goal_msg_.setpoint_odom.resize(2);

    getInput("setpoint_odom_1", goal_msg_.setpoint_odom[0]);
    getInput("setpoint_odom_2", goal_msg_.setpoint_odom[1]);

    getInput("tolerance", goal_msg_.tolerance_odom);
    getInput("timeout", goal_msg_.timeout_odom);

    rclcpp_action::Client<Movement>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&OdomPIDClient::goal_response_callback, this, _1);
    options.feedback_callback =
        std::bind(&OdomPIDClient::feedback_callback, this, _1, _2);
    options.result_callback =
        std::bind(&OdomPIDClient::result_callback, this, _1);

    odom_client_->async_send_goal(goal_msg_, options);

    RCLCPP_INFO(node_->get_logger(), "OdomPIDClient goal sent");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomPIDClient::onRunning()
{
    if (!done_) {
        return BT::NodeStatus::RUNNING;
    }

    return success_
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
}

void OdomPIDClient::onHalted()
{
    cancel_goal();
}

void OdomPIDClient::cancel_goal()
{
    if (goal_handle_) {
        odom_client_->async_cancel_goal(goal_handle_);
        RCLCPP_WARN(node_->get_logger(), "OdomPIDClient goal canceled");
    }
}

void OdomPIDClient::goal_response_callback(
    const GoalHandleMovement::SharedPtr &handle)
{
    if (!handle) {
        RCLCPP_ERROR(node_->get_logger(), "OdomPIDClient goal rejected");
        done_ = true;
        success_ = false;
        return;
    }

    goal_handle_ = handle;
    RCLCPP_INFO(node_->get_logger(), "OdomPIDClient goal accepted");
}

void OdomPIDClient::result_callback(
    const GoalHandleMovement::WrappedResult &result)
{
    success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    done_ = true;

    RCLCPP_INFO(node_->get_logger(),
        "OdomPIDClient result: %s",
        success_ ? "SUCCESS" : "FAILURE");
}

void OdomPIDClient::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const Movement::Feedback>)
{
    // optional
}
