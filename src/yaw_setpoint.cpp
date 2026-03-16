#include "yaw_setpoint.hpp"

YawSetpoint::YawSetpoint(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node){

            rclcpp::QoS qos_profile(10);
            bno_sub = node_->create_subscription<std_msgs::msg::Float32>("/imu_data", sensor_qos(), std::bind(&YawSetpoint::yaw_callback, this, std::placeholders::_1));
            setpoint_pub = node_->create_publisher<std_msgs::msg::Float32MultiArray>("setpoint_yaw",10);
            //slider_sub = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/pid_gains", 10, std::bind(&YawSetpoint::slider_callback, this, std::placeholders::_1));
        }

BT::PortsList YawSetpoint::providedPorts() 
{
    return {
        BT::InputPort<std::string>("turn", "Direction: Right, Left, Back, Front or raw float"),
        BT::BidirectionalPort<CurrentYaw>("current_yaw", CurrentYaw::FRONT, "Current facing direction"),
    };
}

BT::NodeStatus YawSetpoint::onStart(){

    std::string turn_input;
    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();


    getInput<std::string>("turn", turn_input);
    getInput<CurrentYaw>("current_yaw", current_yaw_);

    if      (turn_input == "Right") port_yaw = -1.5707f;
    else if (turn_input == "Left")  port_yaw =  1.5707f;
    else if (turn_input == "Back")  port_yaw =  3.14159f;
    else if (turn_input == "Front") port_yaw =  0.0f;
    else                            port_yaw =  std::stof(turn_input);

    turn_ = imu_data + port_yaw;
    turn_ = wrapAngle(turn_);

    int yaw_int = static_cast<int>(current_yaw_);
    if      (port_yaw == -1.5707f)  yaw_int = (yaw_int + 1) % 4;  // Right
    else if (port_yaw ==  1.5707f)  yaw_int = (yaw_int + 3) % 4;  // Left 
    else if (port_yaw ==  3.14159f) yaw_int = (yaw_int + 2) % 4;  // Back
    current_yaw_ = static_cast<CurrentYaw>(yaw_int);

    setpoint_.data.clear();
    setpoint_.data.push_back(turn_);
    setpoint_.data.push_back(RESET_YAW_BYTE); 
    setpoint_pub->publish(setpoint_);

    
    // setpoint_.data.clear();
    // setpoint_.data.push_back(turn_);
    // setpoint_.data.push_back(RESET_YAW_BYTE); 
    // setpoint_pub->publish(setpoint_);

    // setpoint_.data.clear();
    // setpoint_.data.push_back(turn_);
    // setpoint_.data.push_back(RESET_YAW_BYTE); 
    // setpoint_pub->publish(setpoint_);

    setpoint_.data.clear();
    setpoint_.data.push_back(turn_);
    setpoint_.data.push_back(SET_YAW_BYTE); 
    setpoint_pub->publish(setpoint_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YawSetpoint::onRunning(){

    setpoint_.data.clear();
    setpoint_.data.push_back(turn_);
    setpoint_.data.push_back(SET_YAW_BYTE); 
    setpoint_pub->publish(setpoint_);

    long current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    

    // if(fabs(turn_ - imu_data) < 1.617f && (current_time - start_time) > 200){ 
    //     setpoint_.data.clear();
    //     setpoint_.data.push_back(turn_);
    //     setpoint_.data.push_back(RESET_YAW_BYTE); 
    //     setpoint_pub->publish(setpoint_);
    //     rclcpp::sleep_for(std::chrono::milliseconds(500));

    //     return BT::NodeStatus::SUCCESS;

    // }
    // else{
    //     return BT::NodeStatus::RUNNING;
    // }   

    if(current_time - start_time > 200){
        setpoint_.data.clear();
        setpoint_.data.push_back(turn_);
        setpoint_.data.push_back(RESET_YAW_BYTE); 
        setpoint_pub->publish(setpoint_);
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        setOutput("current_yaw", current_yaw_);
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::RUNNING;
    }
}

void YawSetpoint::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "Yaw Setpoint Halted");
    
}       

void YawSetpoint::yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) {
            imu_data = wrapAngle(msg->data);
            // printf("Current Yaw: %.2f\n", imu_data);
}

void YawSetpoint::slider_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            for (int i = 0; i < 4; i++) {
                slider_data_[i] = msg->data[i];
            }
}