#include "movement_nodes/vel_publish.hpp"

using namespace std::chrono_literals;



VelPublish::VelPublish(const std::string &     name,
           const BT::NodeConfig &  config,
           rclcpp::Node::SharedPtr node_ptr_)
:BT::SyncActionNode(name, config)
, node_ptr_(node_ptr_)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    cmd_vel_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", cmd_vel_qos());
 
}

BT::PortsList VelPublish::providedPorts()
{
    return {
        BT::InputPort<int>("climb_state",
            "Climb state: UP or DOWN"),
        BT::InputPort<int>("next_climb_state",
            "Next node climb state, used for preset roll"),
        
    };
}
BT::NodeStatus VelPublish::tick()
{
    auto climb_state = getInput<int>("climb_state").value();
    if(climb_state == static_cast<int>(ClimbState::UP))
    {
        
        RCLCPP_INFO(node_ptr_->get_logger(), "[VelPublish] moving forward");
        vx = 0.4;
        vy = 0.0;
        vel_publish( vx, vy);
            

    }
    else if(climb_state == static_cast<int>(ClimbState::DOWN))
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[VelPublish]::moving backward");
       
        vx = -0.4;
        vy = 0.0;
        vel_publish(vx, vy);

    }
    else
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "[VelPublish] unknown climb_state=%d", climb_state);
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::SUCCESS;
  
    
   
}
void VelPublish::vel_publish(float vx, float vy)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = vx;
    twist.linear.y  = vy;
    twist.angular.z = 0.0;

    cmd_vel_pub_->publish(twist);
}
// void VelPublish::front_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg)
// {
//     if(msg->data == 1)
//         front_prox_ = true;
//     else
//         front_prox_ = false;

// }
// void VelPublish::rear_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg)
// {
//     if(msg->data == 1)
//         rear_prox_ = true;
//     else
//         rear_prox_ = false;
// }
