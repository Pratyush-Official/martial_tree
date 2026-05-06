#include "zone_and_color.hpp"
using namespace std::placeholders; 

ZoneAndColor::ZoneAndColor(const string &name, const BT::NodeConfiguration &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        team_color_ = node_->create_subscription<std_msgs::msg::String>("team_color", 
            qos_profile, 
            std::bind(&ZoneAndColor::team_color_subscriber_callback, this, _1));

        set_reset_ = node_->create_subscription<std_msgs::msg::UInt8>("set_reset", 
            qos_profile, 
            std::bind(&ZoneAndColor::set_reset_subscriber_callback, this, _1));
        
        reset_to_martial_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("reset_to_martial", 
            qos_profile, 
            std::bind(&ZoneAndColor::reset_to_martial_callback, this, _1));
    }

BT::PortsList ZoneAndColor::providedPorts(){
    return{
        BT::OutputPort<std::string>("color"),
        BT::InputPort<int>("zone_enum"),
        BT::BidirectionalPort<int>("in_arena"),
        BT::BidirectionalPort<int>("in_martial"),
        BT::BidirectionalPort<int>("photo_sequence_flag"),
        BT::BidirectionalPort<int>("in_forest")
    };
}

void ZoneAndColor::team_color_subscriber_callback(const std_msgs::msg::String::SharedPtr msg){
    color_ = msg->data;
}

void ZoneAndColor::set_reset_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    set_reset_val_ = msg->data;
}

void ZoneAndColor::reset_to_martial_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    reset_to_martial_ = msg->data;
}

BT::NodeStatus ZoneAndColor::onStart(){
    return onRunning(); 
}

BT::NodeStatus ZoneAndColor::onRunning(){

    auto res = getInput<int>("in_arena");

    if(color_.empty()){
        return BT::NodeStatus::RUNNING;
    }
    
    setOutput("color", color_);

    if(set_reset_val_ == 0x00){

        if(res.value() == 1){ //When the robot is in arena and reset is called, the "else" condition of the IF_THEN_ELSE node is run, 
             // but if the reset_to_martial button is pressed while in Arena, "then" code of the IF_THEN_ELSE code is run
            if(reset_to_martial_ == 0x01){
                state_ = "success";
                setOutput("in_arena", 0);
                setOutput("photo_sequence_flag", 1);
                setOutput("in_martial", 1);
                setOutput("in_forest", 0);

                return BT::NodeStatus::SUCCESS;
            }
            else{
                state_ = "failure";
                return BT::NodeStatus::FAILURE;
            }
        }

        else{
            state_ = "success";
            return BT::NodeStatus::SUCCESS;
        }
    }

    if(set_reset_val_ == 0x01){
        if(state_ == "success") return BT::NodeStatus::SUCCESS;
        if(state_ == "failure") return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING; 
}

void ZoneAndColor::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "ZoneAndColor halted");
}