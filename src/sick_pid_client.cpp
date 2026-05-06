#include "sick_pid_client.hpp"
 
using namespace std::placeholders;
SickPIDClient::SickPIDClient(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : //name is passes automatically by the bt_xml, its the name = "" thing in the xml
    BT::StatefulActionNode(name, config), node_(node) //config is passed by the bt_xml too for things like ports and shi
{ //bt nodes arent nodes by themselves, so we passa node to make subscribtions, clients etc accessible

        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); 

        client_ptr = rclcpp_action::create_client<ar_interfaces::action::Movement>(node, "/movement_sick"); //movement_pid is the service name
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
        RCLCPP_INFO(node_->get_logger(),"SickPIDClient initialized");
        done_flag = false;
}

// BT::PortsList SickPIDClient::providedPorts()
// {
//     return {
//         BT::InputPort<float>("SP1", "Setpoint 1"),
//         BT::InputPort<float>("SP2", "Setpoint 2"),
//         BT::InputPort<float>("SP3", "Setpoint 3"),
//         BT::InputPort<float>("SP4", "Setpoint 4")
//     };
// }

BT::PortsList SickPIDClient::providedPorts()
{
    return {
        BT::BidirectionalPort<StageDetections>("stage1"),
        BT::InputPort<CurrentYaw>("current_yaw"),
        BT::OutputPort<std::string>("turn", "Direction: Right, Left, Back, Front or raw float"),
        BT::InputPort<std::string>("color", "red or blue")
    };
}

BT::NodeStatus SickPIDClient::onStart()
{
    
    // float sp1 = getInput<float>("SP1").value();
    // float sp2 = getInput<float>("SP2").value();
    // float sp3 = getInput<float>("SP3").value();
    // float sp4 = getInput<float>("SP4").value();


    auto scroll_detections = getInput<StageDetections>("stage1");
    auto current_yaw_ = getInput<CurrentYaw>("current_yaw");
    auto color = getInput<std::string>("color").value_or("blue");

    check = false;
    first_stage_done = false;
    climb_position_reached = false;

    if (!scroll_detections)
    {
    RCLCPP_WARN(node_->get_logger(), "No detections on port: %s",
                scroll_detections.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!current_yaw_) {
        RCLCPP_WARN(node_->get_logger(), "No current_yaw on port");
        return BT::NodeStatus::FAILURE;
    }

    int yaw_int = static_cast<int>(current_yaw_.value());

    std::string turn_output;
    set_setpoints(scroll_detections.value(), yaw_int, turn_output, color);

    if (!turn_output.empty()) {
        setOutput<std::string>("turn", turn_output);
    }

    done_flag = false;
    success = false;
    auto goal_options = rclcpp_action::Client<ar_interfaces::action::Movement>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&SickPIDClient::goal_response_callback, this, _1); //When server sends a response to a goal request, run this callback
    goal_options.feedback_callback = std::bind(&SickPIDClient::feedback_callback, this, _1, _2);
    goal_options.result_callback = std::bind(&SickPIDClient::result_callback, this, _1);

    goal_msg.setpoint_sick = true_setpoint;
    goal_msg.tolerance_sick = 0.0;
    goal_msg.timeout_sick = 0.0;

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
    RCLCPP_INFO(node_->get_logger(), "SickPIDClient::sent goal");
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SickPIDClient::onRunning()
    {

    if (done_flag && success) //Done flag defines whether the execution of the server finished, success defines if the execution was a success or failure.
    {
        check = false;
        auto scroll_detections = getInput<StageDetections>("stage1");
        
        if(scroll_detections){           

            StageDetections det = scroll_detections.value();
            if(under_roi == "roi1"){det.roi1 = "none";} 
            else if(under_roi == "roi2"){det.roi2 = "none";} 
            else if(under_roi == "roi3"){det.roi3 = "none";} 
            setOutput("stage1", det);
        }

        return BT::NodeStatus::SUCCESS;
    }
    
    if (done_flag && !success)
    {
        check = false;
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
        client_ptr->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node_->get_logger(), "SickPIDClient::Goal canceled");
    }
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

// pass turn_output by reference instead of calling setOutput inside
void SickPIDClient::set_setpoints(const StageDetections &det, const int yaw_int_, std::string &turn_output, const std::string &color){

    if (!climb_position_reached && (first_stage_done || (det.roi1 != "real" && det.roi3 != "real"))) { //If roi2 hasnt been reached (i.e real block not on roi2 so roi3 or roi1 needs to be picked up), go to roi2 now
        under_roi = "roi2";

        switch(yaw_int_) {
            case 0:
                true_setpoint = ROI2_SP;
                break;

            case 1:
                true_setpoint = RIGHT_ROI2_SP;
                turn_output = "Left"; //Sends output to port so that when yaw is next executed, the robot is straight
                break;

            case 3:
                true_setpoint = LEFT_ROI2_SP;
                turn_output = "Right";//Sends output to port so that when yaw is next executed, the robot is straight
                break;

        }
        return;
    }

    if (!check) { //Initial check of all the scrolls in Stage 1, if roi2 shows real, other blocks dont need to be picked from entrance zone, if not, they do
        if (det.roi2 == "real") {
            check = true;
            first_stage_done = true;
            climb_position_reached = true;
            under_roi = "roi2";

            switch(yaw_int_) {
                    case 0:
                        true_setpoint = ROI2_SP;
                        break;

                    case 1:
                        true_setpoint = RIGHT_ROI2_SP;
                        turn_output = "Left";
                        break;

                    case 3:
                        true_setpoint = LEFT_ROI2_SP;
                        turn_output = "Right";
                        break;

                }
            return;
        }

        std::string first = (color == "red") ? "roi1" : "roi3"; //If color is red, 1st priority is roi1, if not its roi3
        std::string second = (color == "red") ? "roi3" : "roi1"; //If color is red, 2nd priority is roi3, if not its roi1

        for (const auto &roi : {first, second}) {
            std::string val = (roi == "roi1") ? det.roi1 : det.roi3;
            if (val == "real") {
                check = true;
                first_stage_done = true;
                under_roi = roi;
                bool is_roi1 = (roi == "roi1");

                switch(yaw_int_) {
                    case 0:
                        true_setpoint = is_roi1 ? ROI1_SP : ROI3_SP;
                        break;
                    case 1:
                        true_setpoint = is_roi1 ? RIGHT_ROI1_SP : RIGHT_ROI3_SP;
                        turn_output = "Left";
                        break;
                    case 3:
                        true_setpoint = is_roi1 ? LEFT_ROI1_SP : LEFT_ROI3_SP;
                        turn_output = "Right";
                        break;
                }
                return;
            }
        }
    }
}

void SickPIDClient::resetVel()
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd_pub_->publish(cmd);
}
