
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tree_flow_nodes/conditions/check_for_prox.hpp"
#include "behaviortree_cpp/bt_factory.h"


using namespace BT;
using namespace std::placeholders;

CheckForProx::CheckForProx(
    const std::string &name,
    const BT::NodeConfig &config,
    rclcpp::Node::SharedPtr node
):BT::SyncActionNode(name, config),node_ptr_(node)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    front_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Byte>( "/prox_front_received",  qos_profile,
        std::bind(&CheckForProx::front_subscriber_callback,this,std::placeholders::_1));
    rear_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Byte>( "/prox_rear_received",  qos_profile,
        std::bind(&CheckForProx::rear_subscriber_callback,this,std::placeholders::_1));
    
}

BT::PortsList CheckForProx::providedPorts()
{
    return{
        BT::InputPort<int>("climb_state")  
    };
}
BT::NodeStatus CheckForProx::tick()
{
    auto climb_state_expected = getInput<int>("climb_state");
    // auto climb_state_expected = getInput<int>("climb_state");

    if (!climb_state_expected)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(),
            "[CheckForProx] Missing 'climb_state' input");
        return BT::NodeStatus::FAILURE;
    }

    auto climb_state = climb_state_expected.value();

    // auto climb_state_ = getInput<ClimbState>("climbstate");

    // if(!climb_state_)
    // {
    //     throw BT::RuntimeError("Missing input [climb_state]");
    // }
    // else
    // {
    //     RCLCPP_INFO(node_ptr_->get_logger(),"[CheckForProx]::input");

    // }
    // ClimbState climb_state = climb_state_.value();
    if(climb_state == static_cast<int>(ClimbState::UP))
    {  
        if(front_prox_)
        {
            RCLCPP_INFO(node_ptr_->get_logger(),"[CheckForProx]:: detected front");
            return BT::NodeStatus::SUCCESS;
        }
        
        else
        {  RCLCPP_INFO(node_ptr_->get_logger(),"[CheckForProx]::not front");
            return BT::NodeStatus::FAILURE;
        }
    }
    
    else if(climb_state == static_cast<int>(ClimbState::DOWN))
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[CheckForProx]::rear");
        if(rear_prox_)
            return BT::NodeStatus::FAILURE;
        
        else
            return BT::NodeStatus::SUCCESS;

    }
    RCLCPP_ERROR(node_ptr_->get_logger(), "[CheckForProx] Unknown ClimbState");
    return BT::NodeStatus::FAILURE; 
   
}
void CheckForProx::front_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    if(msg->data == 1)
        front_prox_ = true;
    else
        front_prox_ = false;

}
void CheckForProx::rear_subscriber_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    if(msg->data == 1)
        rear_prox_ = true;
    else
        rear_prox_ = false;
}

namespace BT {
    template <> inline ClimbState convertFromString(StringView str)
    {
        if (str == "UP")   return ClimbState::UP;
        if (str == "DOWN") return ClimbState::DOWN;
        throw RuntimeError(std::string("Invalid ClimbState: ") + str.data());
    }
    template <> inline std::string toStr<ClimbState>(const ClimbState& val)
    {
        return val == ClimbState::UP ? "UP" : "DOWN";
    }
}
