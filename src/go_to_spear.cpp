#include "go_to_spear.hpp"
#include "qos_profiles.hpp"
 
using namespace std::placeholders;
GoToSpear::GoToSpear(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : 
    BT::StatefulActionNode(name, config), node_(node)
{

        client_ptr_spear = rclcpp_action::create_client<ar_interfaces::action::Movement>(node, "/movement_sick"); //movement_pid is the service name

        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
        RCLCPP_INFO(node_->get_logger(),"Go to Spear_Front initialized");
        done_flag = false;
}

BT::PortsList GoToSpear::providedPorts()
{
    return {};
}

BT::NodeStatus GoToSpear::onStart()
{
    done_flag = false;
    success = false;
    goal_handle = nullptr;
    auto goal_options = rclcpp_action::Client<ar_interfaces::action::Movement>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&GoToSpear::goal_response_callback, this, _1); //When server sends a response to a goal request, run node_ callback
    goal_options.feedback_callback = std::bind(&GoToSpear::feedback_callback, this, _1, _2);
    goal_options.result_callback = std::bind(&GoToSpear::result_callback, this, _1);

    goal_msg.setpoint_sick = std::vector<float>{1400.0f, 2880.0f, 0.0f, 0.0f};
    goal_msg.tolerance_sick = 0.0;
    goal_msg.timeout_sick = 0.0;

    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    
    if (!client_ptr_spear->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        done_flag = true;
        success = false;
        return BT::NodeStatus::FAILURE;
    }

    client_ptr_spear->async_send_goal(goal_msg, goal_options);
    RCLCPP_INFO(node_->get_logger(), "GoToSpear::sent goal");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToSpear::onRunning()
    {
    uint32_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();

    if (done_flag){

        if(success)
        {
            RCLCPP_INFO(node_->get_logger(), "Success Sent");
            cmd.linear.x = 0.0f; 
            cmd.linear.y = 0.0f;
            cmd_pub_->publish(cmd);

            return BT::NodeStatus::SUCCESS; 
        }
    
        if (!success)
        {
            RCLCPP_INFO(node_->get_logger(), "Failure Sent");

            cmd.linear.x = 0.0f; 
            cmd.linear.y = 0.0f;
            cmd_pub_->publish(cmd);
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void GoToSpear::onHalted()
{
    cancel_goal();  
}

void GoToSpear::cancel_goal()
{
    if (goal_handle) {
        // Check if the goal is still active before canceling
        auto status = goal_handle->get_status();
        if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || 
            status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) 
        {
            cmd.linear.x = 0.0f; 
            cmd.linear.y = 0.0f;
            cmd_pub_->publish(cmd);
            
            client_ptr_spear->async_cancel_goal(goal_handle);
            RCLCPP_INFO(node_->get_logger(), "GoToSpear::Goal canceled");
        }
    }
}

void GoToSpear::goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_)
{

    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "GoToSpear::Goal rejected by server");
        done_flag = true;
        success = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
                    "GoToSpear::Goal accepted");
        goal_handle = goal_handle_;
    }
    // if (!goal_handle_)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "GoToSpear::Navigate to spear %i goal rejected by server", node_->silo_numbers.data[0]);
    //     setOutput<uint8_t>("Op_SiloNumber", 0);
    // }
    // else
    // {
    //     RCLCPP_INFO(node_->get_logger(), "GoToSpear::Navigating to spear %i ", node_->silo_numbers.data[0]);
    //     node_->goal_handle = goal_handle_;

    // }

}

void GoToSpear::result_callback(const GoalHandleMovement::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "GoToSpear::Goal succeeded");

        cmd.linear.x = 0.0f; 
        cmd.linear.y = 0.0f;
        cmd_pub_->publish(cmd); 
        success = true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "GoToSpear::Goal failed or canceled");

        cmd.linear.x = 0.0f; 
        cmd.linear.y = 0.0f;
        cmd_pub_->publish(cmd);
        success = false;
    }
    done_flag = true;
}

void GoToSpear::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback)
{
    (void)feedback;

}

// void GoToSpear::odometry_callback(const nav_msgs::msg::Odometry &msg)
// {
//     node_->odom_msg = msg;
// }

// void GoToSpear::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     node_->sick_data = msg->data;
// }
