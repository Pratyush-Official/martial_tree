#include "initialize_ports_actuators.hpp"   

InitializePortsActuators::InitializePortsActuators(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos())

    // set_reset = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "set_reset", qos_profile, std::bind(&InitializePortsActuators::set_reset_subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"InitializePortsActuators::Ready..");
}

BT::PortsList InitializePortsActuators::providedPorts()
{
    return {
        BT::OutputPort<int>("in_martial"),
        BT::OutputPort<int>("spear_assembled"),
        BT::OutputPort<int>("scroll_picked_from_starting"),
        BT::OutputPort<int>("climbed_three_photos"),

        BT::OutputPort<int>("in_forest"),
        BT::OutputPort<int>("store"),
        BT::OutputPort<int>("carry"),
        BT::OutputPort<int>("photo_sequence"),
       
        BT::OutputPort<int>("in_arena"),
        BT::OutputPort<>

        };
}

 BT::NodeStatus InitializePortsActuators::tick()
 {  
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    cmd.angular.z = 0.0f;
    cmd_pub_->publish(cmd);
}
