#include "yaw_pid_client.hpp"
 
using namespace std::placeholders;
YawPIDClient::YawPIDClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : 
    BT::StatefulActionNode(name, config), node_(node)
{

        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 

        client_ptr = rclcpp_action::create_client<ar_interfaces::action::Movement>(node, "/yaw_pid"); //movement_pid is the service name
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
        RCLCPP_INFO(node_->get_logger(),"YawPIDClient initialized");
        done_flag = false;
}

BT::PortsList YawPIDClient::providedPorts()
{
    return {
        BT::InputPort<float>("Angle", "Angle setpoint"),
        BT::InputPort<float>("Velocity-max", "Velocity maximum"),
    };
}

BT::NodeStatus YawPIDClient::onStart()
{
    
    float angle = getInput<float>("Angle").value();
    float vel_max = getInput<float>("Velocity-max").value();

    done_flag = false;
    success = false;
    auto goal_options = rclcpp_action::Client<ar_interfaces::action::Movement>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&YawPIDClient::goal_response_callback, this, _1); //When server sends a response to a goal request, run this callback
    goal_options.feedback_callback = std::bind(&YawPIDClient::feedback_callback, this, _1, _2);
    goal_options.result_callback = std::bind(&YawPIDClient::result_callback, this, _1);

    goal_msg.setpoint_yaw = angle;
    goal_msg.tolerance_yaw = 0.0;
    goal_msg.timeout_yaw = 0.0;
    goal_msg.vel_w = vel_max;

    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    
    if (!client_ptr->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        done_flag = true;
        success = false;
        return BT::NodeStatus::FAILURE;
    }

    client_ptr->async_send_goal(goal_msg, goal_options);
    RCLCPP_INFO(node_->get_logger(), "YawPIDClient::sent goal");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YawPIDClient::onRunning()
    {

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

void YawPIDClient::onHalted()
{
    cancel_goal();  
}

void YawPIDClient::cancel_goal()
{
    if (goal_handle)
    {
        client_ptr->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node_->get_logger(), "YawPIDClient::Goal canceled");
    }
}

void YawPIDClient::goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_)
{

    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "YawPIDClient::Goal rejected by server");
        done_flag = true;
        success = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
                    "YawPIDClient::Goal accepted");
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

void YawPIDClient::result_callback(const GoalHandleMovement::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "YawPIDClient::Goal succeeded");
        success = true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "YawPIDClient::Goal failed or canceled");
        success = false;
    }
    done_flag = true;
}

void YawPIDClient::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback)
{
    (void)feedback;

}

void YawPIDClient::resetVel()
{
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z= 0.0f;
    cmd_pub_->publish(cmd);
}
