#include "go_to_forest.hpp"
 
using namespace std::placeholders;
GoToForest::GoToForest(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : 
    BT::StatefulActionNode(name, config), node_(node)
{

        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 

        client_ptr_forest = rclcpp_action::create_client<ar_interfaces::action::Movement>(node, "/movement_pid"); //movement_pid is the service name

        RCLCPP_INFO(node_->get_logger(),"Go to Forest_Front initialized");
        done_flag = false;
}

BT::PortsList GoToForest::providedPorts()
{
    return {
        BT::InputPort<float>("SP1", "Setpoint 1"),
        BT::InputPort<float>("SP2", "Setpoint 2"),
        BT::InputPort<float>("SP3", "Setpoint 3"),
        BT::InputPort<float>("SP4", "Setpoint 4")
    };
}

BT::NodeStatus GoToForest::onStart()
{
    
    float sp1 = getInput<float>("SP1").value();
    float sp2 = getInput<float>("SP2").value();
    float sp3 = getInput<float>("SP3").value();
    float sp4 = getInput<float>("SP4").value();

    done_flag = false;
    success = false;
    auto goal_options = rclcpp_action::Client<ar_interfaces::action::Movement>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&GoToForest::goal_response_callback, this, _1); //When server sends a response to a goal request, run this callback
    goal_options.feedback_callback = std::bind(&GoToForest::feedback_callback, this, _1, _2);
    goal_options.result_callback = std::bind(&GoToForest::result_callback, this, _1);

    goal_msg.setpoint_sick = std::vector<float>{sp1, sp2, sp3, sp4};
    goal_msg.tolerance_sick = 0.0;
    goal_msg.timeout_sick = 0.0;

    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    
    if (!client_ptr_forest->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        done_flag = true;
        success = false;
        return BT::NodeStatus::FAILURE;
    }

    client_ptr_forest->async_send_goal(goal_msg, goal_options);
    RCLCPP_INFO(node_->get_logger(), "GoToForest::sent goal");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToForest::onRunning()
    {
    // Commented timeout logic - check status every tick like GoToSpear
    // if ( now - start_time >= 3000)
    //     {
    //      if( fabs(prev_x - odom_msg.pose.pose.position.x) < 0.01 && fabs((prev_y - odom_msg.pose.pose.position.y)) < 0.01)
    //      {
    //          cancel_goal();
    //          done_flag = true;
    //          is_on_line = false;
    //          RCLCPP_INFO(node_->get_logger(), " RecoveryNode::Robot static.. cancel goal ");
    //          return BT::NodeStatus::SUCCESS;
    //      }
    //      else
    //      {
    //          start_time = get_tick_ms();
    //      }
    //      prev_x = odom_msg.pose.pose.position.x;
    //      prev_y = odom_msg.pose.pose.position.y;
     
    //  if ( fabs(odom_msg.pose.pose.position.y) >= 2.70 && fabs(odom_msg.pose.pose.position.x -  goal_msg.pose.pose.position.x) < 0.3 )
    //  {
    //      if (is_on_line)
    //      {
    //          cancel_goal();

    //         done_flag = true;
    //         is_on_line = false;
    //         x_horiz_line_detected = false;

    //      }
    // }
    
    if (done_flag && success)
    {
        return BT::NodeStatus::SUCCESS; 
    }
    
    if (done_flag && !success)
    {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void GoToForest::onHalted()
{
    cancel_goal();  
}

void GoToForest::cancel_goal()
{
    if (goal_handle)
    {
        client_ptr_forest->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node_->get_logger(), "GoToForest::Goal canceled");
    }
}

void GoToForest::goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_)
{

    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "GoToForest::Goal rejected by server");
        done_flag = true;
        success = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
                    "GoToForest::Goal accepted");
        goal_handle = goal_handle_;
    }
    // if (!goal_handle_)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "GoToSpear::Navigate to spear %i goal rejected by server", this->silo_numbers.data[0]);
    //     setOutput<uint8_t>("Op_SiloNumber", 0);
    // }
    // else
    // {
    //     RCLCPP_INFO(node_->get_logger(), "GoToSpear::Navigating to spear %i ", this->silo_numbers.data[0]);
    //     this->goal_handle = goal_handle_;

    // }

}

void GoToForest::result_callback(const GoalHandleMovement::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "GoToForest::Goal succeeded");
        success = true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "GoToForest::Goal failed or canceled");
        success = false;
    }
    done_flag = true;
}

void GoToForest::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback)
{
    (void)feedback;

}

void GoToForest::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    this->odom_msg = msg;
}

void GoToForest::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    this->sick_data = msg->data;
}
