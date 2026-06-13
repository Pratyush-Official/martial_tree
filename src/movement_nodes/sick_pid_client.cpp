#include "movement_nodes/sick_pid_client.hpp"
 
using namespace std::placeholders;
SickPIDClient::SickPIDClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : 
    BT::StatefulActionNode(name, config), node_(node)
{

        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 

        client_ptr_forest = rclcpp_action::create_client<ar_interfaces::action::Movement>(node, "/movement_sick"); //movement_pid is the service name
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
        RCLCPP_INFO(node_->get_logger(),"SickPIDClient initialized");
        done_flag = false;
}

BT::PortsList SickPIDClient::providedPorts()
{
    return {
        BT::InputPort<float>("SP1", "Setpoint 1"),
        BT::InputPort<float>("SP2", "Setpoint 2"),
        BT::InputPort<float>("SP3", "Setpoint 3"),
        BT::InputPort<float>("SP4", "Setpoint 4")
        // BT::OutputPort<>
    };
}

BT::NodeStatus SickPIDClient::onStart()
{
    
    float sp1 = getInput<float>("SP1").value();
    float sp2 = getInput<float>("SP2").value();
    float sp3 = getInput<float>("SP3").value();
    float sp4 = getInput<float>("SP4").value();

    done_flag = false;
    success = false;
    auto goal_options = rclcpp_action::Client<ar_interfaces::action::Movement>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&SickPIDClient::goal_response_callback, this, _1); //When server sends a response to a goal request, run this callback
    goal_options.feedback_callback = std::bind(&SickPIDClient::feedback_callback, this, _1, _2);
    goal_options.result_callback = std::bind(&SickPIDClient::result_callback, this, _1);

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
    RCLCPP_INFO(node_->get_logger(), "SickPIDClient::sent goal");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SickPIDClient::onRunning()
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

void SickPIDClient::onHalted()
{
    cancel_goal();  
}

void SickPIDClient::cancel_goal()
{
    if (goal_handle)
    {
        client_ptr_forest->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node_->get_logger(), "SickPIDClient::Goal canceled");
    }

    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);  // if you have a cmd_vel publisher here
    
    resetVel();
}

void SickPIDClient::goal_response_callback(const GoalHandleMovement::SharedPtr &goal_handle_)
{

    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "SickPIDClient::Goal rejected by server");
        done_flag = true;
        success = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
                    "SickPIDClient::Goal accepted");
        goal_handle = goal_handle_;
    }
}

void SickPIDClient::result_callback(const GoalHandleMovement::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "SickPIDClient::Goal succeeded");
        success = true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "SickPIDClient::Goal failed or canceled");
        success = false;
    }
    done_flag = true;
}

void SickPIDClient::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const ar_interfaces::action::Movement::Feedback> feedback)
{
    (void)feedback;

}

void SickPIDClient::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    this->odom_msg = msg;
}

void SickPIDClient::sickCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    this->sick_data = msg->data;
}

void SickPIDClient::resetVel()
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd_vel_pub_->publish(cmd);
}