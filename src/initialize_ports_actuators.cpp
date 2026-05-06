#include "initialize_ports_actuators.hpp"   

InitializePortsActuators::InitializePortsActuators(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cmd_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());

    // set_reset = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "set_reset", qos_profile, std::bind(&InitializePortsActuators::set_reset_subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"InitializePortsActuators::Ready..");
}

BT::PortsList InitializePortsActuators::providedPorts()
{
    return {
        BT::BidirectionalPort<int>("in_martial"),
        BT::BidirectionalPort<int>("spear_assembled"),
        BT::BidirectionalPort<int>("scroll_picked_from_entrance"),
        BT::BidirectionalPort<int>("climbed_three_photos"),
        BT::BidirectionalPort<int>("in_forest"),
        BT::BidirectionalPort<int>("store"),
        BT::BidirectionalPort<int>("carry"),
        BT::BidirectionalPort<int>("photo_sequence"),
        BT::BidirectionalPort<int>("in_arena"),
        BT::BidirectionalPort<int>("reset_to_martial")
    };
}

 BT::NodeStatus InitializePortsActuators::tick()
 {  

    flag_alter(FLAG_ALTER_CASE::)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd.angular.z = 0.0f;
    cmd_pub_->publish(cmd);
    return BT::NodeStatus::SUCCESS;
}

void InitializePortsActuators::reset_flags(){

        setOutput("in_martial", 0);
        setOutput("climbed_three_photos", 0);
        setOutput("in_forest", 0);
        setOutput("photo_sequence", 0);
        setOutput("in_arena", 0);
        setOutput("reset_to_martial", 0);
}

void InitializePortsActuators::flag_alter(const FLAG_ALTER_CASE case_){

    auto in_martial                  = getInput<int>("in_martial").value();
    auto spear_assembled             = getInput<int>("spear_assembled").value();
    auto scroll_picked_from_entrance = getInput<int>("scroll_picked_from_entrance").value();
    auto climbed_three_photos        = getInput<int>("climbed_three_photos").value();
    auto in_forest                   = getInput<int>("in_forest").value();
    auto store                       = getInput<int>("store").value();
    auto carry                       = getInput<int>("carry").value();
    auto photo_sequence              = getInput<int>("photo_sequence").value();
    auto in_arena                    = getInput<int>("in_arena").value();
    auto reset_to_martial            = getInput<int>("reset_to_martial").value();
    
    switch(case_){
        case FLAG_ALTER_CASE::GAME_START:

            reset_flags();
            setOutput("in_martial", 1);
            break;

        case FLAG_ALTER_CASE::SPEAR_ASSEMBLED:
            reset_flags();
            setOutput("in_martial", 1);
            setOutput("spear_assembled", 1);
            break;

        case FLAG_ALTER_CASE::SCROLL_PICKED_FROM_ENTRANCE:

            reset_flags();
            setOutput("in_martial", 1);
            setOutput("spear_assembled", 1);
            setOutput("scroll_picked_from_entrance", 1);
            setOutput("store", 1);
            break;

        case FLAG_ALTER_CASE::CLIMBED_THREE_PHOTOS:
            reset_flags();
            setOutput("in_martial", 0);
            setOutput("spear_assembled", 1);
            setOutput("climbed_three_photos", 1);
            setOutput("in_forest", 1);
            break;

        case FLAG_ALTER_CASE::STORE:
            
            setOutput("store", 1);
            break;
        
        case FLAG_ALTER_CASE::CARRY:

            setOutput("carry", 1);
            break;
        
        case FLAG_ALTER_CASE::RESET_IN_FOREST:
            reset_flags();
            setOutput("in_martial", 1);
            setOutput("spear_assembled", 1);
            setOutput("climbed_three_photos", 0);
            setOutput("in_forest", 0);
            setOutput("photo_sequence", 1);
            break;

        case FLAG_ALTER_CASE::RESET_TO_MARTIAL_FROM_ARENA:

            reset_flags();
            setOutput("in_martial", 1);
            setOutput("spear_assembled", 1);
            setOutput("climbed_three_photos", 0);
            setOutput("in_forest", 0);
            setOutput("photo_sequence", 1);
            setOutput("in_arena", 0);
            break;   

        default:
            break;
    }
}
