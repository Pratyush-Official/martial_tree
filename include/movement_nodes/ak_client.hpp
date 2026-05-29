#ifndef AK_CLIENT_HPP
#define AK_CLIENT_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "qos_profiles.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AkClient : public BT::StatefulActionNode
{
public:
    AkClient(const std::string &name,
             const BT::NodeConfig &config,
             rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();
    BT::NodeStatus onRunning() override;
    BT::NodeStatus onStart() override;
    void onHalted() override;

private:

    enum class Movement{     
        UP,
        DOWN,
        IDLE
    };

    enum class Actuation{
        IDLE,
        C_DOWN,
        C_UP,

    };

    Movement movement_state;
    Actuation actuation_state;
    
    bool request_sent_climb;
    bool request_sent_roll;
    bool rear_status_forward; 
    bool rear_status_reverse;

    bool ran;
    bool service_success_climb;
    bool service_done_climb;

    bool service_success_roll;
    bool service_done_roll;
    
    uint8_t prox_data;
    std_msgs::msg::Byte rear;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_c_up;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_c_down;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr active_client_climb;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_roll_forward;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_roll_backward;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr active_client_roll;


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; 
    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr rear_pub;
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr prox_sub;

    geometry_msgs::msg::Twist cmd;

    std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>> future_roll;
    std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>> future_climb;
    
    void prox_callback(const std_msgs::msg::Byte::SharedPtr msg);

    void reset();
    
};

#endif