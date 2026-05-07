#include "initialize_ports_actuators.hpp"   

using namespace std::placeholders;

InitializePortsActuators::InitializePortsActuators(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cmd_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
    team_color_ = node_ptr_->create_subscription<std_msgs::msg::String>("team_color", 
                qos_profile, 
                std::bind(&InitializePortsActuators::team_color_subscriber_callback, this, _1));

    RCLCPP_INFO(node_ptr_->get_logger(), "InitializePortsActuators Initialized");
}

BT::PortsList InitializePortsActuators::providedPorts()
{
    return {
        BT::OutputPort<std::string>("color"),
        BT::OutputPort<std::string>("turn"),
    };
}

BT::NodeStatus InitializePortsActuators::onStart()
{
    return onRunning();
}

BT::NodeStatus InitializePortsActuators::onRunning()
{
    if(color_.empty()){
        return BT::NodeStatus::RUNNING;
    }

    setOutput("color", color_);

    if(color_ == "red") setOutput<std::string>("turn", "Right"); //Setting the yaw value for the initial turn after camera scan
    else if(color_ == "blue") setOutput<std::string>("turn", "Left");

    flag_alter(config().blackboard, FLAG_ALTER_CASE::GAME_START);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd.angular.z = 0.0f;
    cmd_pub_->publish(cmd);

    return BT::NodeStatus::SUCCESS;
}

void InitializePortsActuators::onHalted()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "InitializePortsActuators::Halted");
}

void InitializePortsActuators::reset_flags(BT::Blackboard::Ptr bb){
    bb->set<int>("in_martial", 0);
    bb->set<int>("climbed_three_photos", 0);
    bb->set<int>("in_forest", 0);
    bb->set<int>("photo_sequence", 0);
    bb->set<int>("in_arena", 0);
    bb->set<int>("reset_to_martial", 0);
}

void InitializePortsActuators::team_color_subscriber_callback(const std_msgs::msg::String::SharedPtr msg){
    color_ = msg->data;
}

void InitializePortsActuators::flag_alter(BT::Blackboard::Ptr bb, const FLAG_ALTER_CASE case_){

    switch(case_){
        case FLAG_ALTER_CASE::GAME_START:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            break;

        case FLAG_ALTER_CASE::SPEAR_ASSEMBLED:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            break;

        case FLAG_ALTER_CASE::SCROLL_PICKED_FROM_ENTRANCE:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("scroll_picked_from_entrance", 1);
            bb->set<int>("store", 1);
            break;

        case FLAG_ALTER_CASE::CLIMBED_THREE_PHOTOS:
            reset_flags(bb);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("climbed_three_photos", 1);
            bb->set<int>("in_forest", 1);
            break;

        case FLAG_ALTER_CASE::STORE:
            bb->set<int>("store", 1);
            break;
        
        case FLAG_ALTER_CASE::CARRY:
            bb->set<int>("carry", 1);
            break;
        
        case FLAG_ALTER_CASE::RESET_IN_FOREST:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("photo_sequence", 1);
            break;

        case FLAG_ALTER_CASE::RESET_TO_MARTIAL_FROM_ARENA:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("photo_sequence", 1);
            break;   

        default:
            break;
    }
}