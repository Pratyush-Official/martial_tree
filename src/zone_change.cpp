#include "zone_change.hpp"
using namespace std::placeholders; 

ZoneChange::ZoneChange(const string &name, const BT::NodeConfig &config, const rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        set_reset_ = node_->create_subscription<std_msgs::msg::UInt8>("set_reset", 
            qos_profile, 
            std::bind(&ZoneChange::set_reset_subscriber_callback, this, std::placeholders::_1));
        
        // reset_to_martial_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("reset_to_martial", 
        //     qos_profile, y
        //     std::bind(&ZoneChange::reset_to_martial_callback, this, _1));

        RCLCPP_INFO(node_->get_logger(),"ZoneChange initialized");
    }


BT::PortsList ZoneChange::providedPorts(){
    return{
        
        BT::InputPort<int>("zone_enum"),
        BT::BidirectionalPort<int>("in_arena"),
        BT::BidirectionalPort<int>("in_martial"),
        BT::BidirectionalPort<int>("photo_sequence_flag"),
        BT::BidirectionalPort<int>("in_forest")
    };
}

void ZoneChange::set_reset_subscriber_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    set_reset_val_ = msg->data;
    RCLCPP_INFO(node_->get_logger(), "Button: %d", msg->data);
}

void ZoneChange::reset_to_martial_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    reset_to_martial_ = msg->data;
}

BT::NodeStatus ZoneChange::onStart(){
    return onRunning(); 
}

BT::NodeStatus ZoneChange::onRunning(){

    auto blackboard = config().blackboard;
    int in_arena;

    blackboard->get("in_arena", in_arena);

    if(!in_arena){
        return BT::NodeStatus::RUNNING;
    }

    if(set_reset_val_ == 0){

        if(in_arena == 1){
            //     if(reset_to_martial_ == 0x01){
            //     state_ = "success";
            //     setOutput("in_arena", 0);
            //     setOutput("photo_sequence_flag", 1);
            //     setOutput("in_martial", 1);
            //     setOutput("in_forest", 0);

            //     return BT::NodeStatus::SUCCESS;
            // }
            // else{
                state_ = "failure";
                return BT::NodeStatus::FAILURE;
            // }
            // continue;

        }
        else{
            state_ = "success";
            return BT::NodeStatus::SUCCESS;
        }
    }

    if(set_reset_val_ == 1){
        if(state_ == "success") return BT::NodeStatus::SUCCESS;
        if(state_ == "failure") return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING; 
}

// BT::NodeStatus ZoneChange::onRunning(){s

//     auto res = getInput<int>("in_arena");

//     if(set_reset_val_ == 0){

//         if(res.value() == 1){ //When the robot is in arena and reset is called, the "else" condition of the IF_THEN_ELSE node is run, 
//              // but if the reset_to_martial button is pressed while in Arena, "then" code of the IF_THEN_ELSE code is run
//             // if(reset_to_martial_ == 0x01){
//             //     state_ = "success";
//             //     setOutput("in_arena", 0);
//             //     setOutput("photo_sequence_flag", 1);
//             //     setOutput("in_martial", 1);
//             //     setOutput("in_forest", 0);

//             //     return BT::NodeStatus::SUCCESS;
//             // }
//             // else{
//             //     state_ = "failure";
//             //     return BT::NodeStatus::FAILURE;
//             // }
//             // continue;
//         }

//         else{
//             state_ = "success";
//             return BT::NodeStatus::SUCCESS;
//         }
//     }

//     if(set_reset_val_ == 1){
//         if(state_ == "success") return BT::NodeStatus::SUCCESS;
//         if(state_ == "failure") return BT::NodeStatus::FAILURE;
//     }

//     return BT::NodeStatus::RUNNING; 
// }

void ZoneChange::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "ZoneChange halted");
}