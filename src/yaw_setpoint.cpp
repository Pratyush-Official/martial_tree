#include "yaw_setpoint.hpp"

YawSetpoint::YawSetpoint(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node){

            rclcpp::QoS qos_profile(10);
            bno_sub = node_->create_subscription<std_msgs::msg::Float32>("/imu_data", 10, std::bind(&YawSetpoint::yaw_callback, this, std::placeholders::_1));
            setpoint_pub = node_->create_publisher<std_msgs::msg::Float32MultiArray>("setpoint_yaw", 10);
            //slider_sub = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/pid_gains", 10, std::bind(&YawSetpoint::slider_callback, this, std::placeholders::_1));

        }

BT::PortsList YawSetpoint::providedPorts() 
{
    return {
        BT::InputPort<float>(
            "pi",
            3.14159265358979323846f,
            "Value of pi (can be negative)"
        )
    };
}

BT::NodeStatus YawSetpoint::onStart(){
    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();

    getInput<float>("pi", pi);
    turn_ = pi/2;
    turn_ = wrapAngle(turn_);
    first_time_i = 1;

    printf("Turn Setpoint: %.2f\n", turn_);

    // setpoint_.data.clear();
    // setpoint_.data.push_back(turn_);
    // setpoint_.data.push_back(RESET_YAW_BYTE); 
    // setpoint_pub->publish(setpoint_);

    
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
    
    printf("Turn Setpoint on running: %.2f\n", turn_);

    // if(fabs(yaw - slider_data_[4]) < 20.0f){
    //     SET_YAW_BYTE = 0x00; // stop sending setpoints  
    // }

    // else{
    //     SET_YAW_BYTE = 0x14; // continue sending setpoints
    // }

    // setpoint_.data.clear();
    // setpoint_.data.push_back(90.0f);
    // setpoint_.data.push_back(SET_YAW_BYTE); 
    // setpoint_pub->publish(setpoint_);

    if(fabs(turn_ - imu_data) < 0.8f && (current_time - start_time) > 200){ 
        setpoint_.data.clear();
        setpoint_.data.push_back(turn_);
        setpoint_.data.push_back(RESET_YAW_BYTE); 
        setpoint_pub->publish(setpoint_);
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
            printf("Current Yaw: %.2f\n", imu_data);
}

void YawSetpoint::slider_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            for (int i = 0; i < 4; i++) {
                slider_data_[i] = msg->data[i];
            }
}